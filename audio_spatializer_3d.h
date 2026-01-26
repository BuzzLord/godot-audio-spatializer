/**************************************************************************/
/*  audio_effect_spatializer.h                                            */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include "audio_spatializer.h"

class VelocityTracker3D;
class AudioSpatializer3D;

#ifndef PHYSICS_3D_DISABLED
class Area3D;
#endif // PHYSICS_3D_DISABLED

// Based on "A Novel Multichannel Panning Method for Standard and Arbitrary Loudspeaker Configurations" by Ramy Sadek and Chris Kyriakakis (2004)
// Speaker-Placement Correction Amplitude Panning (SPCAP)
class SpeakerPlacementConfiguration {
private:
	struct Speaker {
		Vector3 direction;
		real_t effective_number_of_speakers = 0; // precalculated
		mutable real_t squared_gain = 0; // temporary
	};

	Vector<Speaker> speakers;

public:
	void update_speaker_configuration(unsigned int speaker_count, const Vector3 *speaker_directions);
	unsigned int get_speaker_count() const;
	Vector3 get_speaker_direction(unsigned int index) const;
	void calculate(const Vector3 &source_direction, real_t tightness, unsigned int volume_count, real_t *volumes) const;
};

class SpatializerParameters3D : public SpatializerParameters {
	GDCLASS(SpatializerParameters3D, SpatializerParameters);

private:
	// Results calculated by AudioEffectSpatializer3D, to be used with mixing

	float linear_attenuation = 0.0; // aka Highshelf gain
	float attenuation_filter_cutoff_hz = 5000.0; // Copied from SpatializerParameters3D per frame
	HashMap<StringName, Vector<Vector2>> bus_volumes;

protected:
	static void _bind_methods();

public:
	void set_linear_attenuation(float p_att);
	float get_linear_attenuation() const;

	void set_attenuation_filter_cutoff_hz(float p_cutoff);
	float get_attenuation_filter_cutoff_hz() const;

	void set_bus_volumes(const HashMap<StringName, Vector<Vector2>> &p_bus_volumes);
	HashMap<StringName, Vector<Vector2>> get_bus_volumes() const;
};

class SpatializerPlaybackData3D : public SpatializerPlaybackData {
	GDCLASS(SpatializerPlaybackData3D, SpatializerPlaybackData);

	Vector<Vector2> prev_mix_volumes;
	AudioFilterSW::Processor filter_processors[8];

protected:
	static void _bind_methods();

public:
	void set_prev_mix_volume(int p_channel, Vector2 p_volume);
	Vector2 get_prev_mix_volume(int p_channel) const;

	AudioFilterSW::Processor *get_filter_processor(int p_channel, bool left);
};

/////////////////////////////////////////////////////////////////////////////////////////

// AudioSpatializerInstance3D
// A concrete implementation of AudioSpatializer/AudioSpatializerInstance that implements the AudioStreamPlayer3D spatializer
class AudioSpatializerInstance3D : public AudioSpatializerInstance {
	GDCLASS(AudioSpatializerInstance3D, AudioSpatializerInstance);
	friend class AudioSpatializer3D;

private:
	Ref<AudioSpatializer3D> base;

	static void _transform_changed_cb(void *self) { reinterpret_cast<AudioSpatializerInstance3D *>(self)->update_doppler_tracked_velocity(); }

	Ref<VelocityTracker3D> velocity_tracker;

	float cached_global_panning_strength = 0.5;

	bool was_further_than_max_distance_last_frame = false;

	bool mix_channel_mode = true;

protected:
	void calc_output_vol_surround(const Vector3 &source_dir, real_t tightness, Vector<Vector2> &output);
	void calc_output_vol_stereo(const Vector3 &source_dir, real_t panning_strength, Vector<Vector2> &output);

	float get_attenuation_db(float p_distance) const;
	void calc_reverb_vol(Area3D *area, Vector3 listener_area_pos, Vector<Vector2> direct_path_vol, Vector<Vector2> &reverb_vol);

	Area3D *_get_overriding_area();
	StringName _get_actual_bus();

public:
	AudioSpatializerInstance3D();
	~AudioSpatializerInstance3D();

	virtual Ref<SpatializerParameters> calculate_spatialization() override;

	virtual void process_frames(Ref<SpatializerParameters> p_parameters, Ref<SpatializerPlaybackData> p_playback_data, AudioFrame *p_output_buf, const AudioFrame *p_source_buf, int p_frame_count) override;

	virtual void mix_channel(Ref<SpatializerParameters> p_parameters, Ref<SpatializerPlaybackData> p_playback_data, int p_channel, AudioFrame *p_output_buf, const AudioFrame *p_source_buf, int p_frame_count) override;
	virtual Ref<SpatializerPlaybackData> instantiate_playback_data() override;
	virtual void initialize_audio_player() override;

	virtual bool should_process_frames() const override { return !mix_channel_mode; }
	virtual bool should_mix_channels() const override { return mix_channel_mode; }

	void update_doppler_tracked_velocity();
};

// AudioSpatializer3D
// A concrete implementation of AudioSpatializer that implements the AudioStreamPlayer3D spatializer
class AudioSpatializer3D : public AudioSpatializer {
	GDCLASS(AudioSpatializer3D, AudioSpatializer);
	friend class AudioSpatializerInstance3D;

	enum AttenuationModel {
		ATTENUATION_INVERSE_DISTANCE,
		ATTENUATION_INVERSE_SQUARE_DISTANCE,
		ATTENUATION_LOGARITHMIC,
		ATTENUATION_DISABLED,
	};

	enum DopplerTracking {
		DOPPLER_TRACKING_DISABLED,
		DOPPLER_TRACKING_IDLE_STEP,
		DOPPLER_TRACKING_PHYSICS_STEP
	};

private:
	AttenuationModel attenuation_model = ATTENUATION_INVERSE_DISTANCE;
	float unit_size = 10.0;
	float max_distance = 0.0;
	float panning_strength = 1.0;

	uint32_t area_mask = 1;

	bool emission_angle_enabled = false;
	float emission_angle = 45.0; // Is in degrees
	float emission_angle_filter_attenuation_db = -12.0;

	float attenuation_filter_cutoff_hz = 5000.0;
	float attenuation_filter_db = -24.0;

	DopplerTracking doppler_tracking = DOPPLER_TRACKING_DISABLED;
	float doppler_speed_of_sound = 343.0;

	bool mix_channel_mode = false;

	SpeakerPlacementConfiguration *spcap = nullptr;

protected:
	static void _bind_methods();

public:
	AudioSpatializer3D();
	~AudioSpatializer3D();

	virtual Ref<AudioSpatializerInstance> instantiate() override;

	// Properties that can be set in editor

	void set_mix_channel_mode(bool p_mode);
	bool get_mix_channel_mode() const;

	void set_unit_size(float p_volume);
	float get_unit_size() const;

	void set_max_distance(float p_metres);
	float get_max_distance() const;

	void set_area_mask(uint32_t p_mask);
	uint32_t get_area_mask() const;

	void set_emission_angle_enabled(bool p_enable);
	bool is_emission_angle_enabled() const;

	void set_emission_angle(float p_angle);
	float get_emission_angle() const;

	void set_emission_angle_filter_attenuation_db(float p_angle_attenuation_db);
	float get_emission_angle_filter_attenuation_db() const;

	void set_attenuation_filter_cutoff_hz(float p_hz);
	float get_attenuation_filter_cutoff_hz() const;

	void set_attenuation_filter_db(float p_db);
	float get_attenuation_filter_db() const;

	void set_attenuation_model(AttenuationModel p_model);
	AttenuationModel get_attenuation_model() const;

	void set_panning_strength(float p_panning_strength);
	float get_panning_strength() const;

	void set_doppler_tracking(DopplerTracking p_tracking);
	DopplerTracking get_doppler_tracking() const;

	void set_doppler_speed_of_sound(float p_speed_mps);
	float get_doppler_speed_of_sound() const;
};

VARIANT_ENUM_CAST(AudioSpatializer3D::AttenuationModel)
VARIANT_ENUM_CAST(AudioSpatializer3D::DopplerTracking)
