/**************************************************************************/
/*  audio_spatializer_effect.h                                            */
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
#include "servers/audio/audio_effect.h"

class AudioSpatializerEffect;

// Implementation of AudioSpatializer that can take a list of AudioEffects, and apply them to the output via process_frames.
class AudioSpatializerInstanceEffect : public AudioSpatializerInstance {
	GDCLASS(AudioSpatializerInstanceEffect, AudioSpatializerInstance);
	friend class AudioSpatializerEffect;

private:
	Ref<AudioSpatializerEffect> base;
	Vector<Ref<AudioEffect>> audio_effects;

	Vector<AudioFrame> temp_buffer;

protected:
	GDVIRTUAL2(_process_effects, Ref<SpatializerParameters>, Ref<SpatializerPlaybackData>)
	static void _bind_methods();

public:
	virtual void process_frames(Ref<SpatializerParameters> p_parameters, Ref<SpatializerPlaybackData> p_playback_data, AudioFrame *p_output_buf, const AudioFrame *p_source_buf, int p_frame_count) override;
	virtual Ref<SpatializerPlaybackData> instantiate_playback_data() override;

	virtual bool should_process_frames() const override { return true; }
	virtual bool should_mix_channels() const override { return false; }

	virtual void process_effects(Ref<SpatializerParameters> p_parameters, Ref<SpatializerPlaybackData> p_playback_data);

	TypedArray<AudioEffect> get_audio_effects() const;
	void set_audio_effects(const TypedArray<AudioEffect> &p_effects);
};

/////////////////////////////////////////////////////////////////////

class SpatializerPlaybackDataEffect : public SpatializerPlaybackData {
	GDCLASS(SpatializerPlaybackDataEffect, SpatializerPlaybackData);

	Vector<Ref<AudioEffectInstance>> effect_instances;

public:
	void add_effect_instance(Ref<AudioEffectInstance> p_instance);
	Vector<Ref<AudioEffectInstance>> get_effect_instances() const;
};

/////////////////////////////////////////////////////////////////////

class AudioSpatializerEffect : public AudioSpatializer {
	GDCLASS(AudioSpatializerEffect, AudioSpatializer);
	friend class AudioSpatializerInstanceEffect;

private:
	LocalVector<Ref<AudioEffect>> audio_effects;

protected:
	GDVIRTUAL0R(Ref<AudioSpatializerInstanceEffect>, _instantiate_effect)
	static void _bind_methods();

public:
	TypedArray<AudioEffect> get_audio_effects() const;
	void set_audio_effects(const TypedArray<AudioEffect> &p_effects);

	virtual Ref<AudioSpatializerInstance> instantiate() override;
};