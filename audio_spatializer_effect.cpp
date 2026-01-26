/**************************************************************************/
/*  audio_spatializer_effect.cpp                                          */
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

#include "audio_spatializer_effect.h"

void AudioSpatializerInstanceEffect::process_frames(Ref<SpatializerParameters> p_parameters, Ref<SpatializerPlaybackData> p_playback_data,
		AudioFrame *p_output_buf, const AudioFrame *p_source_buf, int p_frame_count) {
	ERR_FAIL_COND_MSG(!Object::cast_to<SpatializerPlaybackDataEffect>(*p_playback_data), "Unexpected SpatializerPlaybackData type; expected SpatializerPlaybackDataEffect");
	Ref<SpatializerPlaybackDataEffect> playback_data = Ref<SpatializerPlaybackDataEffect>(Object::cast_to<SpatializerPlaybackDataEffect>(*p_playback_data));
	Vector<Ref<AudioEffectInstance>> effect_instances = playback_data->get_effect_instances();

	process_effects(p_parameters);

	if (effect_instances.size() == 0) {
		for (int frame_idx = 0; frame_idx < p_frame_count; frame_idx++) {
			p_output_buf[frame_idx] = p_source_buf[frame_idx];
		}
		return;
	}

	if (temp_buffer.size() < p_frame_count) {
		temp_buffer.resize(p_frame_count);
	}

	for (int j = 0; j < effect_instances.size(); j++) {
		bool is_even = ((j + effect_instances.size()) % 2 == 0);
		const AudioFrame *src;
		AudioFrame *dst;

		if (j == 0) {
			src = p_source_buf;
		} else {
			if (is_even) {
				src = p_output_buf;
			} else {
				src = temp_buffer.ptr();
			}
		}
		if (is_even) {
			dst = temp_buffer.ptrw();
		} else {
			dst = p_output_buf;
		}

		effect_instances.write[j]->process(src, dst, p_frame_count);
	}
}

Ref<SpatializerPlaybackData> AudioSpatializerInstanceEffect::instantiate_playback_data() {
	Ref<SpatializerPlaybackDataEffect> data;
	data.instantiate();
	for (uint32_t i = 0; i < audio_effects.size(); i++) {
		Ref<AudioEffectInstance> ins = audio_effects[i]->instantiate();
		// ins->set_current_channel(0);
		data->add_effect_instance(ins);
	}
	return data;
}

void AudioSpatializerInstanceEffect::process_effects(Ref<SpatializerParameters> p_parameters) {
	GDVIRTUAL_CALL(_process_effects, p_parameters);
}

TypedArray<AudioEffect> AudioSpatializerInstanceEffect::get_audio_effects() const {
	TypedArray<AudioEffect> arr;

	for (uint32_t i = 0; i < audio_effects.size(); i++) {
		arr.push_back(audio_effects[i]);
	}

	return arr;
}

void AudioSpatializerInstanceEffect::set_audio_effects(const TypedArray<AudioEffect> &p_effects) {
	audio_effects.clear();
	for (int i = 0; i < p_effects.size(); i++) {
		// Cast to proper ref, if our object isn't an AudioEffect resource this will be an empty Ref.
		Ref<AudioEffect> effect = p_effects[i];

		// We add the effect even if this is an empty Ref, this allows the UI to add new entries.
		audio_effects.push_back(effect);
	}
}

void AudioSpatializerInstanceEffect::_bind_methods() {
	GDVIRTUAL_BIND(_process_effects, "spatial_parameters");

	ClassDB::bind_method(D_METHOD("set_audio_effects", "audio_effects"), &AudioSpatializerInstanceEffect::set_audio_effects);
	ClassDB::bind_method(D_METHOD("get_audio_effects"), &AudioSpatializerInstanceEffect::get_audio_effects);

	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "audio_effects", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("AudioEffect")), "set_audio_effects", "get_audio_effects");
}

///////////////////////////////////////////////////////////////////////////////

void SpatializerPlaybackDataEffect::add_effect_instance(Ref<AudioEffectInstance> p_instance) {
	effect_instances.push_back(p_instance);
}

Vector<Ref<AudioEffectInstance>> SpatializerPlaybackDataEffect::get_effect_instances() const {
	return effect_instances;
}

///////////////////////////////////////////////////////////////////////////////

TypedArray<AudioEffect> AudioSpatializerEffect::get_audio_effects() const {
	TypedArray<AudioEffect> arr;

	for (uint32_t i = 0; i < audio_effects.size(); i++) {
		arr.push_back(audio_effects[i]);
	}

	return arr;
}

void AudioSpatializerEffect::set_audio_effects(const TypedArray<AudioEffect> &p_effects) {
	audio_effects.clear();
	for (int i = 0; i < p_effects.size(); i++) {
		// Cast to proper ref, if our object isn't an AudioEffect resource this will be an empty Ref.
		Ref<AudioEffect> effect = p_effects[i];

		// We add the effect even if this is an empty Ref, this allows the UI to add new entries.
		audio_effects.push_back(effect);
	}
}

Ref<AudioSpatializerInstance> AudioSpatializerEffect::instantiate() {
	Ref<AudioSpatializerInstanceEffect> ins;
	GDVIRTUAL_CALL(_instantiate, ins);

	if (ins.is_null()) {
		ins.instantiate();
	}
	ins->base = Ref<AudioSpatializerEffect>(this);
	for (uint32_t i = 0; i < audio_effects.size(); i++) {
		ins->audio_effects.push_back(audio_effects[i]->duplicate());
	}
	return ins;
}

void AudioSpatializerEffect::_bind_methods() {
	GDVIRTUAL_BIND(_instantiate);

	ClassDB::bind_method(D_METHOD("set_audio_effects", "audio_effects"), &AudioSpatializerEffect::set_audio_effects);
	ClassDB::bind_method(D_METHOD("get_audio_effects"), &AudioSpatializerEffect::get_audio_effects);

	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "audio_effects", PROPERTY_HINT_ARRAY_TYPE, MAKE_RESOURCE_TYPE_HINT("AudioEffect")), "set_audio_effects", "get_audio_effects");
}