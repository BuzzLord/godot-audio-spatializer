/**************************************************************************/
/*  audio_effect_spatializer.cpp                                          */
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

#include "audio_spatializer.h"
#include "audio_stream_player_spatial.h"

#include "servers/audio/audio_server.h"

#include "core/config/project_settings.h"
#include "scene/3d/audio_listener_3d.h"
#include "scene/3d/camera_3d.h"
#include "scene/main/viewport.h"

///////////////////////////////////////////////////////////////
// AudioStreamPlayback functions

void AudioSpatializerInstance::start_playback_stream(Ref<AudioStreamPlayback> p_playback, float p_start_time) {
	// print_verbose("AudioSpatializerInstance::start_playback_stream")
	ERR_FAIL_COND(p_playback.is_null());

	//print_verbose("AudioSpatializerInstance start_playback_stream: start");
	Ref<SpatializerParameters> params = get_spatializer_parameters();
	// Check if null for some reason, since get_bus_map uses params below.
	ERR_FAIL_COND(params.is_null());
	// print_verbose("AudioSpatializerInstance start_playback_stream: params good");
	init_channels_and_buffers();

	SpatialPlaybackListNode *playback_node = new SpatialPlaybackListNode();
	playback_node->stream_playback = p_playback;
	playback_node->stream_playback->start(p_start_time);

	//print_verbose("AudioSpatializerInstance start_playback_stream: stream playback started");

	for (AudioFrame &frame : playback_node->lookahead) {
		frame = AudioFrame(0, 0);
	}

	//print_verbose("AudioSpatializerInstance start_playback_stream: lookahead populated");

	playback_node->active.set();
	playback_node->has_frames.set();
	playback_node->playback_data = instantiate_playback_data();

	// print_verbose("AudioSpatializerInstance start_playback_stream: set active and playback data instantiated");

	playback_list.insert(playback_node);

	if (!playback_active.is_set()) {
		playback_active.set();

		for (bool &is_mixed : channel_mixed) {
			is_mixed = true;
		}

		// print_verbose("AudioSpatializerInstance start_playback_stream: playback_active was false, create spatial_playbacks");
		int count = channel_count.get();

		spatial_playbacks.resize(count);
		for (int channel_idx = 0; channel_idx < count; channel_idx++) {
			Ref<AudioStreamPlaybackSpatial> spatial_playback;
			spatial_playback.instantiate();
			spatial_playback->active.set();
			spatial_playback->channel = channel_idx;
			spatial_playback->spatializer = this;
			spatial_playbacks.write[channel_idx] = spatial_playback;
			AudioServer::get_singleton()->start_playback_stream(spatial_playback, get_bus_map(params, channel_idx));
		}
	}
}

void AudioSpatializerInstance::stop_playback_stream(Ref<AudioStreamPlayback> p_playback) {
	ERR_FAIL_COND(p_playback.is_null());

	// print_verbose("AudioSpatializerInstance stop_playback_stream");
	if (p_playback->is_playing()) {
		p_playback->stop();
	}

	SpatialPlaybackListNode *playback_node = _find_playback_list_node(p_playback);
	if (!playback_node) {
		return;
	}

	// print_verbose("AudioSpatializerInstance stop_playback_stream: playback_node found, setting inactive");
	playback_node->active.clear();
}

void AudioSpatializerInstance::set_playback_paused(bool p_paused) {
	// Pause spatial_playbacks on AudioServer
	for (int channel_idx = 0; channel_idx < spatial_playbacks.size(); channel_idx++) {
		if (spatial_playbacks[channel_idx]->active.is_set()) {
			AudioServer::get_singleton()->set_playback_paused(spatial_playbacks[channel_idx], p_paused);
		}
	}
}

bool AudioSpatializerInstance::is_playback_active(Ref<AudioStreamPlayback> p_playback) {
	ERR_FAIL_COND_V(p_playback.is_null(), false);

	// if (p_playback->get_is_sample()) {
	// 	if (p_playback->get_sample_playback().is_valid()) {
	// 		return sample_playback_list.has(p_playback->get_sample_playback());
	// 	} else {
	// 		return false;
	// 	}
	// }

	SpatialPlaybackListNode *playback_node = _find_playback_list_node(p_playback);
	if (!playback_node) {
		return false;
	}
	if (!playback_active.is_set()) {
		return false;
	}
	return playback_node->active.is_set();
}

float AudioSpatializerInstance::get_playback_position(Ref<AudioStreamPlayback> p_playback) {
	ERR_FAIL_COND_V(p_playback.is_null(), 0);

	// Samples.
	// if (p_playback->get_is_sample() && p_playback->get_sample_playback().is_valid()) {
	// 	Ref<AudioSamplePlayback> sample_playback = p_playback->get_sample_playback();
	// 	return AudioServer::get_singleton()->get_sample_playback_position(sample_playback);
	// }

	SpatialPlaybackListNode *playback_node = _find_playback_list_node(p_playback);
	if (!playback_node) {
		return 0;
	}
	return playback_node->stream_playback->get_playback_position();
}

bool AudioSpatializerInstance::is_playback_paused() {
	// Paused applies to spatializer as a whole. Check the first spatial_playback state from AudioServer.
	if (!playback_active.is_set()) {
		return false;
	}
	if (spatial_playbacks.is_empty()) {
		return false;
	}
	return AudioServer::get_singleton()->is_playback_paused(spatial_playbacks[0]);
}

void AudioSpatializerInstance::init_channels_and_buffers() {
	//print_verbose("AudioSpatializerInstance init_channels_and_buffers");
	int count;
	if (should_mix_channels()) {
		count = AudioServer::get_singleton()->get_channel_count();
	} else {
		count = 1;
	}

	int old_count = channel_count.get();
	if (old_count != count) {
		channel_count.set(count);
		// print_verbose(vformat("AudioSpatializerInstance: channel_count set to %d", channel_count.get()));

		if (playback_active.is_set()) {
			Ref<SpatializerParameters> params = get_spatializer_parameters();
			if (params.is_null()) {
				return;
			}

			// Mark all channels as mixed, so next process frame will update mix_buffer
			for (bool &is_mixed : channel_mixed) {
				is_mixed = true;
			}

			if (count > old_count) {
				spatial_playbacks.resize(count);
				for (int channel_idx = old_count; channel_idx < count; channel_idx++) {
					Ref<AudioStreamPlaybackSpatial> spatial_playback;
					spatial_playback.instantiate();
					spatial_playback->active.set();
					spatial_playback->channel = channel_idx;
					spatial_playback->spatializer = this;
					spatial_playbacks.write[channel_idx] = spatial_playback;
					AudioServer::get_singleton()->start_playback_stream(spatial_playback, get_bus_map(params, channel_idx));
				}
			} else {
				for (int channel_idx = old_count - 1; channel_idx >= count; channel_idx--) {
					AudioServer::get_singleton()->stop_playback_stream(spatial_playbacks[channel_idx]);
				}
				spatial_playbacks.resize(count);
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

Ref<SpatializerParameters> AudioSpatializerInstance::calculate_spatialization() {
	Ref<SpatializerParameters> ret;
	GDVIRTUAL_CALL(_calculate_spatialization, ret);
	return ret;
}

bool AudioSpatializerInstance::should_process_frames() const {
	bool ret = false;
	GDVIRTUAL_CALL(_should_process_frames, ret);
	return ret;
}

void AudioSpatializerInstance::process_frames(Ref<SpatializerParameters> p_spatial_parameters, Ref<SpatializerPlaybackData> p_playback_data,
		AudioFrame *p_output_buf, const AudioFrame *p_source_buf, int p_frame_count) {
	GDVIRTUAL_CALL(_process_frames, p_spatial_parameters, p_playback_data, p_output_buf, p_source_buf, p_frame_count);
}

bool AudioSpatializerInstance::should_mix_channels() const {
	bool ret = false;
	GDVIRTUAL_CALL(_should_mix_channels, ret);
	return ret;
}

void AudioSpatializerInstance::mix_channel(Ref<SpatializerParameters> p_spatial_parameters, Ref<SpatializerPlaybackData> p_playback_data, int p_channel,
		AudioFrame *p_output_buf, const AudioFrame *p_source_buf, int p_frame_count) {
	GDVIRTUAL_CALL(_mix_channel, p_spatial_parameters, p_playback_data, p_channel, p_output_buf, p_source_buf, p_frame_count);
}

Ref<SpatializerPlaybackData> AudioSpatializerInstance::instantiate_playback_data() {
	Ref<SpatializerPlaybackData> ret;
	GDVIRTUAL_CALL(_instantiate_playback_data, ret);
	return ret;
}

void AudioSpatializerInstance::initialize_audio_player() {
	GDVIRTUAL_CALL(_initialize_audio_player);
}

void AudioSpatializerInstance::update_spatializer_parameters() {
	//print_verbose("Calculate new spatialization");
	Ref<SpatializerParameters> new_parameters = calculate_spatialization();
	//print_verbose("Done calculating new spatialization");
	ERR_FAIL_COND_MSG(new_parameters.is_null(), "SpatializerParameters from calculate_spatialization() cannot be null");
	set_spatializer_parameters(new_parameters);

	if (new_parameters->should_update_parameters()) {
		for (int channel_idx = 0; channel_idx < spatial_playbacks.size(); channel_idx++) {
			Ref<AudioStreamPlayback> playback = spatial_playbacks[channel_idx];
			HashMap<StringName, Vector<AudioFrame>> bus_map = get_bus_map(new_parameters, channel_idx);
			AudioServer::get_singleton()->set_playback_bus_volumes_linear(playback, bus_map);
		}
	}
}

HashMap<StringName, Vector<AudioFrame>> AudioSpatializerInstance::get_bus_map(const Ref<SpatializerParameters> p_params, int p_channel) {
	//print_verbose(vformat("AudioSpatializerInstance get_bus_map: channel %d", p_channel));

	HashMap<StringName, Vector<AudioFrame>> bus_map;
	ERR_FAIL_INDEX_V(p_channel, MAX_CHANNELS_PER_BUS, bus_map);

	int idx = 0;
	Dictionary bus_volumes = p_params->get_bus_volumes();
	Vector<Vector2> mix_volumes = p_params->get_mix_volumes();

	for (StringName key : bus_volumes.get_key_list()) {
		if (idx >= MAX_BUSES_PER_PLAYBACK) {
			break;
		}

		Vector<AudioFrame> volumes;
		volumes.resize(MAX_CHANNELS_PER_BUS);

		Vector<Vector2> bus_volume = bus_volumes.get_valid(key);
		bus_map[key] = volumes;

		if (should_mix_channels()) {
			// Mix masked out volumes for specified channel:
			for (int channel_idx = 0; channel_idx < MAX_CHANNELS_PER_BUS; channel_idx++) {
				float left = 0.0;
				float right = 0.0;
				if (channel_idx == p_channel) {
					// Mask out only the requested channel index

					// Normalize bus volumes by the mix volumes
					if (mix_volumes[channel_idx][0] > 0.0) {
						left = bus_volume[channel_idx][0] / mix_volumes[channel_idx][0];
					}
					if (mix_volumes[channel_idx][1] > 0.0) {
						right = bus_volume[channel_idx][1] / mix_volumes[channel_idx][1];
					}
				}

				bus_map[key].write[channel_idx] = AudioFrame(left, right);
			}
		} else {
			// Not mixing channels ourselves, pass all mix volumes out for AudioServer to mix.
			for (int channel_idx = 0; channel_idx < MAX_CHANNELS_PER_BUS; channel_idx++) {
				bus_map[key].write[channel_idx] = mix_volumes[channel_idx];
			}
		}

		idx++;
	}
	return bus_map;
}

void AudioSpatializerInstance::_mix_from_playback_list(int p_buffer_size) {
	// print_verbose("AudioSpatializerInstance _mix_from_playback_list: Start mixing stream playbacks");
	Ref<SpatializerParameters> parameters = get_spatializer_parameters();
	//print_verbose(vformat("Got parameters: %s", parameters.is_null() ? "null" : "valid"));
	ERR_FAIL_COND(parameters.is_null());

	if (temp_buffer.size() < p_buffer_size) {
		temp_buffer.resize(p_buffer_size);
	}
	for (int c = 0; c < channel_count.get(); c++) {
		if (mix_buffer[c].size() < p_buffer_size) {
			mix_buffer.write[c].resize(p_buffer_size);
		}
		AudioFrame *channel_buf = mix_buffer.write[c].ptrw();
		for (int i = 0; i < p_buffer_size; i++) {
			channel_buf[i] = AudioFrame(0.f, 0.f);
		}
	}

	int64_t buffer_lookahead_size = p_buffer_size + LOOKAHEAD_BUFFER_SIZE;
	if (playback_buffer.size() < buffer_lookahead_size) {
		playback_buffer.resize(buffer_lookahead_size);
	}
	if (process_buffer.size() < p_buffer_size) {
		process_buffer.resize(p_buffer_size);
	}

	for (SpatialPlaybackListNode *playback : playback_list) {
		// Inactive streams are no-ops. Don't even mix audio from the stream playback.
		if (!playback->active.is_set()) {
			continue;
		}

		if (playback->stream_playback->get_is_sample()) {
			continue;
		}

		// print_verbose("AudioSpatializerInstance _mix_from_playback_list: mixing playback");

		Ref<SpatializerPlaybackData> playback_data = playback->playback_data;

		AudioFrame *buf = playback_buffer.ptrw();

		if (playback->has_frames.is_set()) {
			// Copy the old contents of the lookahead buffer into the beginning of the playback buffer.
			for (int i = 0; i < LOOKAHEAD_BUFFER_SIZE; i++) {
				buf[i] = playback->lookahead[i];
			}

			float pitch_scale = parameters->get_pitch_scale();

			// Mix frame_count from the playback into the stored mixed_buffer for the playback
			int mixed_frames = playback->stream_playback->mix(&buf[LOOKAHEAD_BUFFER_SIZE], pitch_scale, p_buffer_size);

			if (mixed_frames != p_buffer_size) {
				// We know we have at least the size of our lookahead buffer for fade-out purposes.
				float fadeout_base = 0.96;
				float fadeout_coefficient = 1;
				float buffer_size_float = (float)LOOKAHEAD_BUFFER_SIZE;
				float buffer_linear_fade_idx = 0.0;

				int fade_limit = mixed_frames + LOOKAHEAD_BUFFER_SIZE;
				for (int idx = mixed_frames; idx < p_buffer_size; idx++) {
					if (idx < fade_limit) {
						fadeout_coefficient *= fadeout_base;
						buf[idx] *= fadeout_coefficient * (buffer_size_float - buffer_linear_fade_idx) / buffer_size_float;
						buffer_linear_fade_idx += 1.0;
					} else {
						buf[idx] *= 0.0;
					}
				}
				// No more frames to mix
				playback->has_frames.clear();
			} else {
				// Move the last little bit of what we just mixed into our lookahead buffer for the next time we mix playback frames.
				for (int i = 0; i < LOOKAHEAD_BUFFER_SIZE; i++) {
					playback->lookahead[i] = buf[p_buffer_size + i];
				}
			}
		} else {
			// No more frames to mix, fill with zeros to let process_frames/mix_channel fill in.
			playback_buffer.fill(AudioFrame(0, 0));
		}

		const AudioFrame *processed_buf;
		if (should_process_frames()) {
			// Do any non-channel specific processing on the playback buffer.
			process_frames(parameters, playback_data, process_buffer.ptrw(), buf, p_buffer_size);
			processed_buf = process_buffer.ptr();
		} else {
			processed_buf = buf;
		}

		AudioFrame peak = AudioFrame(0, 0);

		if (should_mix_channels()) {
			int channels = channel_count.get();
			// print_verbose(vformat("AudioSpatializerInstance _mix_from_playback_list %d channels", channels));
			for (int channel_idx = 0; channel_idx < channels; channel_idx++) {
				AudioFrame *output_buf = temp_buffer.ptrw();

				// Mixes p_frame_count frames from processed buffer into channel buffer, using spatial calculation parameters
				// and persistent playback_data.
				mix_channel(parameters, playback_data, channel_idx, output_buf, processed_buf, p_buffer_size);

				// Mix the buffer into the channel, find this playbacks peak.
				AudioFrame *channel_buf = mix_buffer.write[channel_idx].ptrw();
				for (int i = 0; i < p_buffer_size; i++) {
					channel_buf[i] += output_buf[i];

					float l = Math::abs(output_buf[i].left);
					if (l > peak.left) {
						peak.left = l;
					}
					float r = Math::abs(output_buf[i].right);
					if (r > peak.right) {
						peak.right = r;
					}
				}
			}
		} else {
			// print_verbose(vformat("AudioSpatializerInstance _mix_from_playback_list no mix channels"));
			// Copy processed buffer into (channel 0) output buffer, and find peak.
			AudioFrame *output_buf = mix_buffer.write[0].ptrw();
			for (int i = 0; i < p_buffer_size; i++) {
				output_buf[i] += processed_buf[i];

				float l = Math::abs(processed_buf[i].left);
				if (l > peak.left) {
					peak.left = l;
				}
				float r = Math::abs(processed_buf[i].right);
				if (r > peak.right) {
					peak.right = r;
				}
			}
		}

		if (!playback->has_frames.is_set()) {
			if (MAX(peak.right, peak.left) <= Math::db_to_linear(playback_disable_threshold_db)) {
				playback->active.clear();
				// print_verbose(vformat("Playback stopped: %.1f,%.1f <= %.1f", Math::linear_to_db(peak.right), Math::linear_to_db(peak.left), playback_disable_threshold_db));
			}
		}
	}
}

void AudioSpatializerInstance::_manage_playback_state() {
	int active_playbacks = 0;
	for (SpatialPlaybackListNode *playback : playback_list) {
		active_playbacks++;
		if (!playback->active.is_set()) {
			// print_verbose(vformat("AudioSpatializerInstance _manage_playback_state: inactive node on playback list being deleted"));
			_delete_stream_playback_list_node(playback);
			active_playbacks--;
		}
	}

	if (active_playbacks == 0) {
		// print_verbose("AudioSpatializerInstance _manage_playback_state: no active nodes found, cleaning up spatial");
		for (int i = 0; i < spatial_playbacks.size(); i++) {
			AudioServer::get_singleton()->stop_playback_stream(spatial_playbacks[i]);
		}
		spatial_playbacks.clear();
		playback_active.clear();
	}
}

bool AudioSpatializerInstance::_check_channel_mixed(int p_channel) {
	if (channel_mixed[p_channel]) {
		// If this channel has already been mixed, then we need more data from the playbacks.
		// Reset everything to false, then set p_channel to true (since we'll be mixing it now)
		for (bool &is_mixed : channel_mixed) {
			is_mixed = false;
		}
		channel_mixed[p_channel] = true;
		return true;
	} else {
		// We haven't been mixed yet, so mark it true for next time.
		channel_mixed[p_channel] = true;
		return false;
	}
}

void AudioSpatializerInstance::get_mixed_frames(int p_channel, AudioFrame *p_frames, int p_frame_count) {
	//ERR_FAIL_COND(!is_playback_paused());

	//print_verbose(vformat("AudioSpatializerInstance get_mixed_frames %d", p_channel));
	init_channels_and_buffers();

	if (_check_channel_mixed(p_channel)) {
		_mix_from_playback_list(p_frame_count);
		_manage_playback_state();
	}

	ERR_FAIL_INDEX_MSG(p_channel, mix_buffer.size(), vformat("Unexpected channel %d", p_channel));
	ERR_FAIL_COND_MSG(p_frame_count != mix_buffer[p_channel].size(), vformat("Unexpected frame count %d", p_frame_count));
	const AudioFrame *buf = mix_buffer[p_channel].ptr();
	for (int i = 0; i < p_frame_count; i++) {
		p_frames[i] = buf[i];
	}
}

AudioSpatializerInstance::SpatialPlaybackListNode *AudioSpatializerInstance::_find_playback_list_node(Ref<AudioStreamPlayback> p_playback) {
	for (SpatialPlaybackListNode *playback_list_node : playback_list) {
		if (playback_list_node->stream_playback == p_playback) {
			return playback_list_node;
		}
	}
	return nullptr;
}

void AudioSpatializerInstance::_delete_stream_playback_list_node(SpatialPlaybackListNode *p_playback_node) {
	// Remove the playback from the list, registering a destructor to be run on the main thread.
	playback_list.erase(p_playback_node, [](SpatialPlaybackListNode *p) {
		p->stream_playback.unref();
		p->playback_data.unref();
		// active is fine
		// lookahead is fine
		delete p;
	});
}

void AudioSpatializerInstance::set_audio_player(AudioStreamPlayerSpatial *p_audio_player) {
	audio_player = p_audio_player;
	initialize_audio_player();
}

AudioStreamPlayerSpatial *AudioSpatializerInstance::get_audio_player() const {
	return audio_player;
}

void AudioSpatializerInstance::set_spatializer_parameters(Ref<SpatializerParameters> p_parameters) {
	mutex.lock();
	//print_verbose("AudioSpatializerInstance set_spatializer_parameters start")
	spatializer_parameters = p_parameters;
	//print_verbose("AudioSpatializerInstance set_spatializer_parameters complete")
	mutex.unlock();
}

Ref<SpatializerParameters> AudioSpatializerInstance::get_spatializer_parameters() const {
	Ref<SpatializerParameters> ret;
	mutex.lock();
	//print_verbose("AudioSpatializerInstance get_spatializer_parameters start")
	ret = spatializer_parameters;
	//print_verbose("AudioSpatializerInstance get_spatializer_parameters complete")
	mutex.unlock();
	return ret;
}

float AudioSpatializerInstance::get_playback_disable_threshold_db() const {
	return playback_disable_threshold_db;
}

void AudioSpatializerInstance::set_playback_disable_threshold_db(float p_threshold) {
	playback_disable_threshold_db = p_threshold;
}

void AudioSpatializerInstance::_bind_methods() {
	GDVIRTUAL_BIND(_calculate_spatialization);
	GDVIRTUAL_BIND(_should_process_frames);
	GDVIRTUAL_BIND(_process_frames, "spatial_parameters", "playback_data", "output_buf", "source_buf", "frame_count");
	GDVIRTUAL_BIND(_should_mix_channels);
	GDVIRTUAL_BIND(_mix_channel, "spatial_parameters", "playback_data", "channel", "output_buf", "source_buf", "frame_count");
	GDVIRTUAL_BIND(_instantiate_playback_data);
	GDVIRTUAL_BIND(_initialize_audio_player);

	ClassDB::bind_method(D_METHOD("get_audio_player"), &AudioSpatializerInstance::get_audio_player);

	ClassDB::bind_method(D_METHOD("set_playback_disable_threshold_db", "threshold"), &AudioSpatializerInstance::set_playback_disable_threshold_db);
	ClassDB::bind_method(D_METHOD("get_playback_disable_threshold_db"), &AudioSpatializerInstance::get_playback_disable_threshold_db);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "playback_disable_threshold_db", PROPERTY_HINT_RANGE, "-80,0,0.1,suffix:dB"), "set_playback_disable_threshold_db", "get_playback_disable_threshold_db");
}

AudioSpatializerInstance::AudioSpatializerInstance() {
	mix_buffer.resize(MAX_CHANNELS_PER_BUS);
	channel_count.set(0);
	playback_active.clear();
}

///////////////////////////////////////////////////////////////////////////////

Ref<AudioSpatializerInstance> AudioSpatializer::instantiate() {
	Ref<AudioSpatializerInstance> ret;
	GDVIRTUAL_CALL(_instantiate, ret);
	return ret;
}

void AudioSpatializer::_bind_methods() {
	GDVIRTUAL_BIND(_instantiate);
}

AudioSpatializer::AudioSpatializer() {
}

///////////////////////////////////////////////////////////////////////////////

void AudioStreamPlaybackSpatial::start(double p_from_pos) {
	// print_verbose(vformat("AudioStreamPlaybackSpatial[%d]: start(%f) [%s]", channel, p_from_pos, active.is_set() ? "active" : "inactive"));
	// if (active.is_set()) {
	// 	if (stream_playback->is_playing()) {
	// 		double playback_pos = stream_playback->get_playback_position();
	// 		if (playback_pos != p_from_pos) {
	// 			// maybe do something?
	// 			//print_verbose(vformat("AudioStreamPlaybackSpatial[%d]: start called on playing stream at %f vs new start pos %f", channel, playback_pos, p_from_pos));
	// 		}
	// 	} else {
	// 		stream_playback->start(p_from_pos);
	// 	}
	// }
	//
}

void AudioStreamPlaybackSpatial::stop() {
	// print_verbose(vformat("AudioStreamPlaybackSpatial[%d]: stop() [%s]", channel, active.is_set() ? "active" : "inactive"));
	// if (active.is_set()) {
	// 	if (stream_playback->is_playing()) {
	// 		stream_playback->stop();
	// 	}
	// }
}

bool AudioStreamPlaybackSpatial::is_playing() const {
	// Expect this to get called every AudioServer frame
	// print_verbose(vformat("AudioStreamPlaybackSpatial[%d]: is_playing() [%s]", channel, active.is_set() ? "active" : "inactive"));
	// if (active.is_set()) {
	// 	return stream_playback->is_playing();
	// }
	return spatializer->playback_active.is_set();
}

int AudioStreamPlaybackSpatial::get_loop_count() const {
	//print_verbose(vformat("AudioStreamPlaybackSpatial[%d]: get_loop_count() [%s]", channel, active.is_set() ? "active" : "inactive"));
	return 0;
}

double AudioStreamPlaybackSpatial::get_playback_position() const {
	// Only called when get_playback_position called
	//print_verbose(vformat("AudioStreamPlaybackSpatial[%d]: get_playback_position() [%s]", channel, active.is_set() ? "active" : "inactive"));
	// if (active.is_set()) {
	// 	return stream_playback->get_playback_position();
	// }
	// return spatializer->get_playback_position();
	return 0;
}

void AudioStreamPlaybackSpatial::tag_used_streams() {
	// Expect this to get called every AudioServer frame
	//print_verbose(vformat("AudioStreamPlaybackSpatial[%d]: tag_used_streams() [%s]", channel, active ? "active" : "inactive"));
	// if (active.is_set() && channel == 0) {
	// 	stream_playback->tag_used_streams();
	// }
}

int AudioStreamPlaybackSpatial::mix(AudioFrame *p_buffer, float p_rate_scale, int p_frames) {
	// Expect this to get called every AudioServer frame
	if (spatializer->playback_active.is_set() && active.is_set()) {
		//print_verbose(vformat("AudioStreamPlaybackSpatial mix %d", channel));
		// Ignore p_rate_scale; it comes from the playback node in AudioServer, which is set by us anyway, we have a local copy we'll use.
		spatializer->get_mixed_frames(channel, p_buffer, p_frames);
		return p_frames;
	} else {
		return 0;
	}
}
