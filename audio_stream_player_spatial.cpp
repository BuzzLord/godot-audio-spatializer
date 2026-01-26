/**************************************************************************/
/*  audio_stream_player_spatial.cpp                                       */
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

#include "audio_stream_player_spatial.h"
#include "audio_spatializer.h"
#include "spatializer_parameters.h"

#include "core/config/project_settings.h"
#include "scene/3d/audio_listener_3d.h"
#include "scene/3d/camera_3d.h"
#include "scene/main/viewport.h"
#include "servers/audio/audio_stream.h"

#ifndef PHYSICS_3D_DISABLED
#include "scene/3d/physics/area_3d.h"
#endif // PHYSICS_3D_DISABLED


void AudioStreamPlayerSpatial::_notification(int p_what) {
	//print_verbose(vformat("AudioStreamPlayerSpatial _notification %d", p_what));
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			AudioServer::get_singleton()->add_listener_changed_callback(_listener_changed_cb, this);
			if (spatializer_base.is_valid() && spatializer.is_null()) {
				//print_verbose("AudioStreamPlayerSpatial instantiate new spatializer");
				spatializer = spatializer_base->instantiate();
				spatializer->set_audio_player(this);
			}
			
			if (autoplay && !Engine::get_singleton()->is_editor_hint()) {
				play(0.0);
			}
			set_stream_paused(!can_process());
		} break;

		case NOTIFICATION_EXIT_TREE: {
			AudioServer::get_singleton()->remove_listener_changed_callback(_listener_changed_cb, this);
			set_stream_paused(true);
		} break;

		case NOTIFICATION_READY: {
		} break;
		
		case NOTIFICATION_TRANSFORM_CHANGED: {
			notify_transform_changed();
		} break;

		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {

			// Update anything related to position first, if possible of course.
			if (spatializer.is_valid() && (active.is_set() || setplay.get() > 0 || force_update_spatializer)) {
				force_update_spatializer = false;
				spatializer->update_spatializer_parameters();
			}

			if (spatializer.is_valid() && setplayback.is_valid() && setplay.get() >= 0) {
				active.set();
				
				spatializer->start_playback_stream(setplayback, setplay.get());
				setplayback.unref();
				setplay.set(-1);
			}

			if (!stream_playbacks.is_empty() && active.is_set()) {
				process_playbacks();
			}
			ensure_playback_limit();
		} break;

		case Node::NOTIFICATION_PREDELETE: {
			stream_playbacks.clear();
		} break;

		case Node::NOTIFICATION_SUSPENDED:
		case Node::NOTIFICATION_PAUSED: {
			if (!can_process()) {
				// Node can't process so we start fading out to silence
				set_stream_paused(true);
			}
		} break;

		case Node::NOTIFICATION_UNSUSPENDED: {
			if (get_tree()->is_paused()) {
				break;
			}
			[[fallthrough]];
		}

		case Node::NOTIFICATION_UNPAUSED: {
			set_stream_paused(false);
		} break;
	}
}

void AudioStreamPlayerSpatial::process_playbacks() {
	if (spatializer.is_null()) {
		return;
	}
	Vector<Ref<AudioStreamPlayback>> playbacks_to_remove;
	for (Ref<AudioStreamPlayback> &playback : stream_playbacks) {
		if (playback.is_valid() && !spatializer->is_playback_active(playback) && !spatializer->is_playback_paused(playback)) {
			playbacks_to_remove.push_back(playback);
		}
	}
	// Now go through and remove playbacks that have finished. Removing elements from a Vector in a range based for is asking for trouble.
	for (Ref<AudioStreamPlayback> &playback : playbacks_to_remove) {
		stream_playbacks.erase(playback);
	}
	if (!playbacks_to_remove.is_empty() && stream_playbacks.is_empty()) {
		// This node is no longer actively playing audio.
		active.clear();
		_set_process(false);
	}
	if (!playbacks_to_remove.is_empty()) {
		emit_signal(SceneStringName(finished));
	}
}

void AudioStreamPlayerSpatial::ensure_playback_limit() {
	if (spatializer.is_null()) {
		return;
	}
	while (stream_playbacks.size() > max_polyphony) {
		spatializer->stop_playback_stream(stream_playbacks[0]);
		stream_playbacks.remove_at(0);
	}
}

void AudioStreamPlayerSpatial::_set_process(bool p_enabled) {
	set_physics_process_internal(p_enabled);
}

void AudioStreamPlayerSpatial::_update_stream_parameters() {
	if (stream.is_null()) {
		return;
	}

	List<AudioStream::Parameter> parameters;
	stream->get_parameter_list(&parameters);
	for (const AudioStream::Parameter &K : parameters) {
		const PropertyInfo &pi = K.property;
		StringName key = PARAM_PREFIX + pi.name;
		if (!playback_parameters.has(key)) {
			ParameterData pd = {pi.name, K.default_value};
			playback_parameters.insert(key, pd);
		}
	}
}

void AudioStreamPlayerSpatial::set_stream(Ref<AudioStream> p_stream) {
	if (stream.is_valid()) {
		stream->disconnect(SNAME("parameter_list_changed"), callable_mp(this, &AudioStreamPlayerSpatial::_update_stream_parameters));
	}
	stop();
	stream = p_stream;
	_update_stream_parameters();
	if (stream.is_valid()) {
		stream->connect(SNAME("parameter_list_changed"), callable_mp(this, &AudioStreamPlayerSpatial::_update_stream_parameters));
	}
	notify_property_list_changed();
}

Ref<AudioStream> AudioStreamPlayerSpatial::get_stream() const {
	return stream;
}

void AudioStreamPlayerSpatial::set_volume_db(float p_volume) {
	ERR_FAIL_COND_MSG(Math::is_nan(p_volume), "Volume can't be set to NaN.");
	volume_db = p_volume;
}

float AudioStreamPlayerSpatial::get_volume_db() const {
	return volume_db;
}

void AudioStreamPlayerSpatial::set_volume_linear(float p_volume) {
	set_volume_db(Math::linear_to_db(p_volume));
}

float AudioStreamPlayerSpatial::get_volume_linear() const {
	return Math::db_to_linear(get_volume_db());
}

void AudioStreamPlayerSpatial::set_max_db(float p_boost) {
	max_db = p_boost;
}

float AudioStreamPlayerSpatial::get_max_db() const {
	return max_db;
}

void AudioStreamPlayerSpatial::set_pitch_scale(float p_pitch_scale) {
	pitch_scale = p_pitch_scale;
}

float AudioStreamPlayerSpatial::get_pitch_scale() const {
	return pitch_scale;
}

void AudioStreamPlayerSpatial::play(float p_from_pos) {
	if (spatializer.is_null()) {
		return;
	}
	Ref<AudioStreamPlayback> stream_playback = _play_basic();
	if (stream_playback.is_null()) {
		return;
	}
	//print_verbose(vformat("AudioStreamPlayerSpatial play from pos %f", p_from_pos));
	setplayback = stream_playback;
	setplay.set(p_from_pos);

	// Sample handling.
	// if (stream_playback->get_is_sample() && stream_playback->get_sample_playback().is_valid()) {
	// 	Ref<AudioSamplePlayback> sample_playback = stream_playback->get_sample_playback();
	// 	sample_playback->offset = p_from_pos;
	// 	sample_playback->bus = _get_actual_bus();

	// 	AudioServer::get_singleton()->start_sample_playback(sample_playback);
	// }
}

Ref<AudioStreamPlayback> AudioStreamPlayerSpatial::_play_basic() {
	//print_verbose("AudioStreamPlayerSpatial play_basic");
	Ref<AudioStreamPlayback> stream_playback;
	if (stream.is_null()) {
		//print_verbose("AudioStreamPlayerSpatial play_basic: stream is null");
		return stream_playback;
	}
	ERR_FAIL_COND_V_MSG(!is_inside_tree(), stream_playback, "Playback can only happen when a node is inside the scene tree");
	if (stream->is_monophonic() && is_playing()) {
		//print_verbose("AudioStreamPlayerSpatial play_basic: stream is monophonic, stopping all playbacks");
		stop();
	}
	stream_playback = stream->instantiate_playback();
	ERR_FAIL_COND_V_MSG(stream_playback.is_null(), stream_playback, "Failed to instantiate playback.");

	for (const KeyValue<StringName, ParameterData> &K : playback_parameters) {
		stream_playback->set_parameter(K.value.path, K.value.value);
	}

	// Sample handling.
	// if (_is_sample()) {
	// 	if (stream->can_be_sampled()) {
	// 		stream_playback->set_is_sample(true);
	// 		if (stream_playback->get_is_sample() && stream_playback->get_sample_playback().is_null()) {
	// 			if (!AudioServer::get_singleton()->is_stream_registered_as_sample(stream)) {
	// 				AudioServer::get_singleton()->register_stream_as_sample(stream);
	// 			}
	// 			Ref<AudioSamplePlayback> sample_playback;
	// 			sample_playback.instantiate();
	// 			sample_playback->stream = stream;
	// 			sample_playback->pitch_scale = pitch_scale;
	// 			stream_playback->set_sample_playback(sample_playback);
	// 		}
	// 	} else if (!stream->is_meta_stream()) {
	// 		WARN_PRINT(vformat(R"(%s is trying to play a sample from a stream that cannot be sampled.)", node->get_path()));
	// 	}
	// }

	//print_verbose("AudioStreamPlayerSpatial play_basic: Adding new stream playback to list, returning it");
	stream_playbacks.push_back(stream_playback);
	active.set();
	_set_process(true);
	return stream_playback;
}

void AudioStreamPlayerSpatial::seek(float p_seconds) {
	if (spatializer.is_null()) {
		return;
	}
	if (is_playing()) {
		stop();
		play(p_seconds);
	}
}

void AudioStreamPlayerSpatial::stop() {
	if (spatializer.is_null()) {
		return;
	}
	
	//print_verbose("AudioStreamPlayerSpatial::stop")
	setplay.set(-1);
	_stop_basic();
}

void AudioStreamPlayerSpatial::_stop_basic() {
	
	//print_verbose("AudioStreamPlayerSpatial stop_basic");
	for (Ref<AudioStreamPlayback> &playback : stream_playbacks) {
		spatializer->stop_playback_stream(playback);
	}
	stream_playbacks.clear();

	active.clear();
	_set_process(false);
}

bool AudioStreamPlayerSpatial::is_playing() const {
	if (spatializer.is_null()) {
		return false;
	}
	if (setplay.get() >= 0) {
		return true; // play() has been called this frame, but no playback exists just yet.
	}
	for (const Ref<AudioStreamPlayback> &playback : stream_playbacks) {
		if (spatializer->is_playback_active(playback)) {
			return true;
		}
	}
	return false;
}

float AudioStreamPlayerSpatial::get_playback_position() {
	if (spatializer.is_null()) {
		return 0;
	}
	if (setplay.get() >= 0) {
		return setplay.get(); // play() has been called this frame, but no playback exists just yet.
	}
	// Return the playback position of the most recently started playback stream.
	if (!stream_playbacks.is_empty()) {
		return spatializer->get_playback_position(stream_playbacks[stream_playbacks.size() - 1]);
	}
	return 0;
}

void AudioStreamPlayerSpatial::set_autoplay(bool p_enable) {
	autoplay = p_enable;
}

bool AudioStreamPlayerSpatial::is_autoplay_enabled() const {
	return autoplay;
}

void AudioStreamPlayerSpatial::_set_playing(bool p_enable) {
	if (spatializer.is_null()) {
		return;
	}
	if (p_enable) {
		play(0.0);
	} else {
		stop();
	}
}


void AudioStreamPlayerSpatial::set_stream_paused(bool p_pause) {
	if (spatializer.is_null()) {
		return;
	}
	// TODO this does not have perfect recall, fix that maybe? If there are zero playbacks registered with the AudioServer, this bool isn't persisted.
	for (Ref<AudioStreamPlayback> &playback : stream_playbacks) {
		spatializer->set_playback_paused(playback, p_pause);
		// if (_is_sample() && playback->get_sample_playback().is_valid()) {
		// 	AudioServer::get_singleton()->set_sample_playback_pause(playback->get_sample_playback(), p_pause);
		// }
	}
}

bool AudioStreamPlayerSpatial::get_stream_paused() const {
	if (spatializer.is_null()) {
		return false;
	}
	// There's currently no way to pause some playback streams but not others. Check the first and don't bother looking at the rest.
	if (!stream_playbacks.is_empty()) {
		return spatializer->is_playback_paused(stream_playbacks[0]);
	}
	return false;
}

bool AudioStreamPlayerSpatial::has_stream_playback() {
	if (spatializer.is_null()) {
		return false;
	}
	return !stream_playbacks.is_empty();
}

Ref<AudioStreamPlayback> AudioStreamPlayerSpatial::get_stream_playback() {
	ERR_FAIL_COND_V_MSG(stream_playbacks.is_empty(), Ref<AudioStreamPlayback>(), "Player is inactive. Call play() before requesting get_stream_playback().");
	return stream_playbacks[stream_playbacks.size() - 1];
}

void AudioStreamPlayerSpatial::set_bus(const StringName &p_bus) {
	bus = p_bus; // This will be pushed to the audio server during the next physics timestep, which is fast enough.
}

StringName AudioStreamPlayerSpatial::get_bus() const {
	const String bus_name = bus;
	for (int i = 0; i < AudioServer::get_singleton()->get_bus_count(); i++) {
		if (AudioServer::get_singleton()->get_bus_name(i) == bus_name) {
			return bus;
		}
	}
	return SceneStringName(Master);
}

void AudioStreamPlayerSpatial::set_max_polyphony(int p_max_polyphony) {
	if (p_max_polyphony > 0) {
		max_polyphony = p_max_polyphony;
	}
}

int AudioStreamPlayerSpatial::get_max_polyphony() const {
	return max_polyphony;
}

bool AudioStreamPlayerSpatial::_set(const StringName &p_name, const Variant &p_value) {
	ParameterData *pd = playback_parameters.getptr(p_name);
	if (!pd) {
		return false;
	}
	pd->value = p_value;
	for (Ref<AudioStreamPlayback> &playback : stream_playbacks) {
		playback->set_parameter(pd->path, pd->value);
	}
	return true;
}

bool AudioStreamPlayerSpatial::_get(const StringName &p_name, Variant &r_ret) const {
	const ParameterData *pd = playback_parameters.getptr(p_name);
	if (!pd) {
		return false;
	}
	r_ret = pd->value;
	return true;
}

void AudioStreamPlayerSpatial::_get_property_list(List<PropertyInfo> *p_list) const {
	if (stream.is_null()) {
		return;
	}
	List<AudioStream::Parameter> parameters;
	stream->get_parameter_list(&parameters);
	for (const AudioStream::Parameter &K : parameters) {
		PropertyInfo pi = K.property;
		pi.name = PARAM_PREFIX + pi.name;

		const ParameterData *pd = playback_parameters.getptr(pi.name);
		if (pd && pd->value == K.default_value) {
			pi.usage &= ~PROPERTY_USAGE_STORAGE;
		}

		p_list->push_back(pi);
	}
}

void AudioStreamPlayerSpatial::_validate_property(PropertyInfo &p_property) const {
	if (!Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	if (p_property.name == "bus") {
		String options;
		for (int i = 0; i < AudioServer::get_singleton()->get_bus_count(); i++) {
			if (i > 0) {
				options += ",";
			}
			String name = AudioServer::get_singleton()->get_bus_name(i);
			options += name;
		}

		p_property.hint_string = options;
	}
}

void AudioStreamPlayerSpatial::set_spatializer(Ref<AudioSpatializer> p_spatializer) {
	//print_verbose("AudioStreamPlayerSpatial::set_spatializer");
	spatializer_base = p_spatializer;
	if (spatializer.is_valid()) {
		//print_verbose("AudioStreamPlayerSpatial unref old spatializer");
		spatializer.unref();
	}
}

Ref<AudioSpatializer> AudioStreamPlayerSpatial::get_spatializer() const {
	return spatializer_base;
}

void AudioStreamPlayerSpatial::_bind_methods() {

	ClassDB::bind_method(D_METHOD("set_spatializer", "spatializer"), &AudioStreamPlayerSpatial::set_spatializer);
	ClassDB::bind_method(D_METHOD("get_spatializer"), &AudioStreamPlayerSpatial::get_spatializer);

	ClassDB::bind_method(D_METHOD("set_stream", "stream"), &AudioStreamPlayerSpatial::set_stream);
	ClassDB::bind_method(D_METHOD("get_stream"), &AudioStreamPlayerSpatial::get_stream);

	ClassDB::bind_method(D_METHOD("set_volume_db", "volume_db"), &AudioStreamPlayerSpatial::set_volume_db);
	ClassDB::bind_method(D_METHOD("get_volume_db"), &AudioStreamPlayerSpatial::get_volume_db);

	ClassDB::bind_method(D_METHOD("set_volume_linear", "volume_linear"), &AudioStreamPlayerSpatial::set_volume_linear);
	ClassDB::bind_method(D_METHOD("get_volume_linear"), &AudioStreamPlayerSpatial::get_volume_linear);

	ClassDB::bind_method(D_METHOD("set_max_db", "max_db"), &AudioStreamPlayerSpatial::set_max_db);
	ClassDB::bind_method(D_METHOD("get_max_db"), &AudioStreamPlayerSpatial::get_max_db);

	ClassDB::bind_method(D_METHOD("set_pitch_scale", "pitch_scale"), &AudioStreamPlayerSpatial::set_pitch_scale);
	ClassDB::bind_method(D_METHOD("get_pitch_scale"), &AudioStreamPlayerSpatial::get_pitch_scale);

	ClassDB::bind_method(D_METHOD("play", "from_position"), &AudioStreamPlayerSpatial::play, DEFVAL(0.0));
	ClassDB::bind_method(D_METHOD("seek", "to_position"), &AudioStreamPlayerSpatial::seek);
	ClassDB::bind_method(D_METHOD("stop"), &AudioStreamPlayerSpatial::stop);

	ClassDB::bind_method(D_METHOD("is_playing"), &AudioStreamPlayerSpatial::is_playing);
	ClassDB::bind_method(D_METHOD("get_playback_position"), &AudioStreamPlayerSpatial::get_playback_position);

	ClassDB::bind_method(D_METHOD("set_autoplay", "enable"), &AudioStreamPlayerSpatial::set_autoplay);
	ClassDB::bind_method(D_METHOD("is_autoplay_enabled"), &AudioStreamPlayerSpatial::is_autoplay_enabled);

	ClassDB::bind_method(D_METHOD("set_playing", "enable"), &AudioStreamPlayerSpatial::_set_playing);

	ClassDB::bind_method(D_METHOD("set_stream_paused", "pause"), &AudioStreamPlayerSpatial::set_stream_paused);
	ClassDB::bind_method(D_METHOD("get_stream_paused"), &AudioStreamPlayerSpatial::get_stream_paused);

	ClassDB::bind_method(D_METHOD("set_bus", "bus"), &AudioStreamPlayerSpatial::set_bus);
	ClassDB::bind_method(D_METHOD("get_bus"), &AudioStreamPlayerSpatial::get_bus);

	ClassDB::bind_method(D_METHOD("set_max_polyphony", "max_polyphony"), &AudioStreamPlayerSpatial::set_max_polyphony);
	ClassDB::bind_method(D_METHOD("get_max_polyphony"), &AudioStreamPlayerSpatial::get_max_polyphony);

	ClassDB::bind_method(D_METHOD("has_stream_playback"), &AudioStreamPlayerSpatial::has_stream_playback);
	ClassDB::bind_method(D_METHOD("get_stream_playback"), &AudioStreamPlayerSpatial::get_stream_playback);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "stream", PROPERTY_HINT_RESOURCE_TYPE, "AudioStream"), "set_stream", "get_stream");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "volume_db", PROPERTY_HINT_RANGE, "-80,80,suffix:dB"), "set_volume_db", "get_volume_db");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "volume_linear", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "set_volume_linear", "get_volume_linear");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_db", PROPERTY_HINT_RANGE, "-24,6,suffix:dB"), "set_max_db", "get_max_db");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pitch_scale", PROPERTY_HINT_RANGE, "0.01,4,0.01,or_greater"), "set_pitch_scale", "get_pitch_scale");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "playing", PROPERTY_HINT_ONESHOT, "", PROPERTY_USAGE_EDITOR), "set_playing", "is_playing");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "autoplay"), "set_autoplay", "is_autoplay_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "stream_paused", PROPERTY_HINT_NONE, ""), "set_stream_paused", "get_stream_paused");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_polyphony", PROPERTY_HINT_NONE, ""), "set_max_polyphony", "get_max_polyphony");
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "bus", PROPERTY_HINT_ENUM, ""), "set_bus", "get_bus");

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "spatializer", PROPERTY_HINT_RESOURCE_TYPE, "AudioSpatializer"), "set_spatializer", "get_spatializer");

	ADD_SIGNAL(MethodInfo("finished"));
}

void AudioStreamPlayerSpatial::notify_transform_changed() {
	for (CallbackItem *ci : transform_changed_callback_list) {
		ci->callback(ci->userdata);
	}
}

void AudioStreamPlayerSpatial::add_transform_changed_callback(AudioCallback p_callback, void *p_userdata) {
	set_notify_transform(true);
	
	CallbackItem *ci = new CallbackItem();
	ci->callback = p_callback;
	ci->userdata = p_userdata;
	transform_changed_callback_list.insert(ci);
}

void AudioStreamPlayerSpatial::remove_transform_changed_callback(AudioCallback p_callback, void *p_userdata) {
	int callback_count = 0;
	for (CallbackItem *ci : transform_changed_callback_list) {
		callback_count++;
		if (ci->callback == p_callback && ci->userdata == p_userdata) {
			callback_count--;
			transform_changed_callback_list.erase(ci, [](CallbackItem *c) { delete c; });
		}
	}
	if (callback_count == 0) {
		set_notify_transform(false);
	}
}

AudioStreamPlayerSpatial::AudioStreamPlayerSpatial() {
	//internal = memnew(AudioStreamPlayerInternal(this, callable_mp(this, &AudioStreamPlayerSpatial::play), callable_mp(this, &AudioStreamPlayerSpatial::stop), true));

	set_disable_scale(true);
	//cached_global_panning_strength = GLOBAL_GET_CACHED(float, "audio/general/3d_panning_strength");

	bus = SceneStringName(Master);
	AudioServer::get_singleton()->connect("bus_layout_changed", callable_mp((Object *)this, &Object::notify_property_list_changed));
	AudioServer::get_singleton()->connect("bus_renamed", callable_mp((Object *)this, &Object::notify_property_list_changed).unbind(3));
}

AudioStreamPlayerSpatial::~AudioStreamPlayerSpatial() {
	//memdelete(internal);
}
