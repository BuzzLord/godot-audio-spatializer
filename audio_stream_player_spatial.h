/**************************************************************************/
/*  audio_stream_player_spatial.h                                         */
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

#include "scene/3d/node_3d.h"
#include "servers/audio/audio_server.h"

struct AudioFrame;
class AudioStream;
class AudioStreamPlayback;

class AudioSpatializer;
class AudioSpatializerInstance;
class SpatializerParameters;

class AudioStreamPlayerSpatial : public Node3D {
	GDCLASS(AudioStreamPlayerSpatial, Node3D);

public:

	typedef void (*AudioCallback)(void *p_userdata);

private:
	enum {
		MAX_OUTPUTS = 8,
		MAX_INTERSECT_AREAS = 32

	};

	SafeNumeric<float> setplay{ -1.0 };
	Ref<AudioStreamPlayback> setplayback;

	float max_db = 3.0;

	uint64_t last_mix_count = -1;
	bool force_update_spatializer = false;

	Ref<AudioSpatializer> spatializer_base;
	Ref<AudioSpatializerInstance> spatializer;
	Ref<SpatializerParameters> spatializer_parameters;

	static void _listener_changed_cb(void *self) { reinterpret_cast<AudioStreamPlayerSpatial *>(self)->force_update_spatializer = true; }

	void _set_playing(bool p_enable);
	bool _is_active() const;

	struct ParameterData {
		StringName path;
		Variant value;
	};

	static inline const String PARAM_PREFIX = "parameters/";

	HashMap<StringName, ParameterData> playback_parameters;

	void _set_process(bool p_enabled);
	void _update_stream_parameters();

	Ref<AudioStreamPlayback> _play_basic();
	void _stop_basic();

	struct CallbackItem {
		AudioCallback callback;
		void *userdata = nullptr;
	};

	SafeList<CallbackItem *> transform_changed_callback_list;
	
	Vector<Ref<AudioStreamPlayback>> stream_playbacks;
	Ref<AudioStream> stream;
	
	SafeFlag active;
	
	float pitch_scale = 1.0;
	float volume_db = 0.0;
	bool autoplay = false;
	StringName bus;
	int max_polyphony = 1;

protected:
	void _validate_property(PropertyInfo &p_property) const;
	void _notification(int p_what);
	static void _bind_methods();

	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;

	void process_playbacks();
	void ensure_playback_limit();

public:
	
	void set_spatializer(Ref<AudioSpatializer> p_spatializer);
	Ref<AudioSpatializer> get_spatializer() const;

	void set_stream(Ref<AudioStream> p_stream);
	Ref<AudioStream> get_stream() const;

	void set_volume_db(float p_volume);
	float get_volume_db() const;

	void set_volume_linear(float p_volume);
	float get_volume_linear() const;

	void set_max_db(float p_boost);
	float get_max_db() const;

	void set_pitch_scale(float p_pitch_scale);
	float get_pitch_scale() const;

	void set_bus(const StringName &p_bus);
	StringName get_bus() const;

	void set_max_polyphony(int p_max_polyphony);
	int get_max_polyphony() const;

	void set_autoplay(bool p_enable);
	bool is_autoplay_enabled() const;

	void play(float p_from_pos = 0.0);
	void seek(float p_seconds);
	void stop();
	bool is_playing() const;
	float get_playback_position();

	void set_stream_paused(bool p_pause);
	bool get_stream_paused() const;

	bool has_stream_playback();
	Ref<AudioStreamPlayback> get_stream_playback();

	// AudioServer::PlaybackType get_playback_type() const;
	// void set_playback_type(AudioServer::PlaybackType p_playback_type);

	void notify_transform_changed();
	void add_transform_changed_callback(AudioCallback p_callback, void *p_userdata);
	void remove_transform_changed_callback(AudioCallback p_callback, void *p_userdata);

	AudioStreamPlayerSpatial();
	~AudioStreamPlayerSpatial();
};
