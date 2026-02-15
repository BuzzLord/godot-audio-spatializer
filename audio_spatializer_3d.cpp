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

#include "audio_spatializer_3d.h"
#include "audio_stream_player_spatial.h"

#include "servers/audio/audio_server.h"
#include "servers/audio/audio_stream.h"

#include "core/config/project_settings.h"
#include "scene/3d/audio_listener_3d.h"
#include "scene/3d/camera_3d.h"
#include "scene/3d/velocity_tracker_3d.h"
#include "scene/main/viewport.h"

#ifndef PHYSICS_3D_DISABLED
#include "scene/3d/physics/area_3d.h"
#endif // PHYSICS_3D_DISABLED

static const Vector3 default_speaker_directions[7] = {
	Vector3(-1.0, 0.0, -1.0).normalized(), // front-left
	Vector3(1.0, 0.0, -1.0).normalized(), // front-right
	Vector3(0.0, 0.0, -1.0).normalized(), // center
	Vector3(-1.0, 0.0, 1.0).normalized(), // rear-left
	Vector3(1.0, 0.0, 1.0).normalized(), // rear-right
	Vector3(-1.0, 0.0, 0.0).normalized(), // side-left
	Vector3(1.0, 0.0, 0.0).normalized(), // side-right
};

void AudioSpatializerInstance3D::calc_output_vol_surround(const Vector3 &source_dir, real_t tightness, Vector<Vector2> &output) {
	unsigned int speaker_count = 0; // only main speakers (no LFE)
	switch (AudioServer::get_singleton()->get_speaker_mode()) {
		case AudioServer::SPEAKER_MODE_STEREO:
			speaker_count = 2;
			break;
		case AudioServer::SPEAKER_SURROUND_31:
			speaker_count = 3;
			break;
		case AudioServer::SPEAKER_SURROUND_51:
			speaker_count = 5;
			break;
		case AudioServer::SPEAKER_SURROUND_71:
			speaker_count = 7;
			break;
	}

	if (base->spcap->get_speaker_count() != speaker_count) {
		base->spcap->update_speaker_configuration(speaker_count, default_speaker_directions);
	}
	real_t volumes[7];
	base->spcap->calculate(source_dir, tightness, speaker_count, volumes);

	switch (AudioServer::get_singleton()->get_speaker_mode()) {
		case AudioServer::SPEAKER_SURROUND_71:
			output.write[3][0] = volumes[5]; // side-left
			output.write[3][1] = volumes[6]; // side-right
			[[fallthrough]];
		case AudioServer::SPEAKER_SURROUND_51:
			output.write[2][0] = volumes[3]; // rear-left
			output.write[2][1] = volumes[4]; // rear-right
			[[fallthrough]];
		case AudioServer::SPEAKER_SURROUND_31:
			output.write[1][0] = volumes[2]; // center
			output.write[1][1] = 1.0; // LFE - always full power
			[[fallthrough]];
		case AudioServer::SPEAKER_MODE_STEREO:
			output.write[0][0] = volumes[0]; // front-left
			output.write[0][1] = volumes[1]; // front-right
			break;
	}
}

// Set the volume to cosine of half horizontal the angle from the source to the left/right speaker direction ignoring elevation.
// Then scale `cosx` so that greatest ratio of the speaker volumes is `1-panning_strength`.
// See https://github.com/godotengine/godot/issues/103989 for evidence that this is the most standard implementation.
void AudioSpatializerInstance3D::calc_output_vol_stereo(const Vector3 &source_dir, real_t pan_strength, Vector<Vector2> &output) {
	double flatrad = sqrt(source_dir.x * source_dir.x + source_dir.z * source_dir.z);
	double g = CLAMP((1.0 - pan_strength) * (1.0 - pan_strength), 0.0, 1.0);
	double f = (1.0 - g) / (1.0 + g);
	double cosx = CLAMP(source_dir.x / (flatrad == 0.0 ? 1.0 : flatrad), -1.0, 1.0);
	double fcosx = cosx * f;
	output.write[0] = Vector2(sqrt((-fcosx + 1.0) / 2.0), sqrt((fcosx + 1.0) / 2.0));
}

void AudioSpatializerInstance3D::calc_output_vol(const Vector3 &source_dir, Vector<Vector2> &output) {
	if (AudioServer::get_singleton()->get_speaker_mode() == AudioServer::SPEAKER_MODE_STEREO) {
		calc_output_vol_stereo(source_dir, cached_global_panning_strength * base->panning_strength, output);
	} else {
		// Bake in a constant factor here to allow the project setting defaults for 2d and 3d to be normalized to 1.0.
		float tightness = cached_global_panning_strength * 2.0f;
		tightness *= base->panning_strength;
		calc_output_vol_surround(source_dir, tightness, output);
	}
}

float AudioSpatializerInstance3D::get_attenuation_db(float p_distance) const {
	float att = 0;
	switch (base->attenuation_model) {
		case AudioSpatializer3D::ATTENUATION_INVERSE_DISTANCE: {
			att = Math::linear_to_db(1.0 / ((p_distance / base->unit_size) + CMP_EPSILON));
		} break;
		case AudioSpatializer3D::ATTENUATION_INVERSE_SQUARE_DISTANCE: {
			float d = (p_distance / base->unit_size);
			d *= d;
			att = Math::linear_to_db(1.0 / (d + CMP_EPSILON));
		} break;
		case AudioSpatializer3D::ATTENUATION_LOGARITHMIC: {
			att = -20 * Math::log(p_distance / base->unit_size + CMP_EPSILON);
		} break;
		case AudioSpatializer3D::ATTENUATION_DISABLED:
			break;
		default: {
			ERR_PRINT("Unknown attenuation type");
			break;
		}
	}

	att += get_audio_player()->get_volume_db();
	if (att > get_audio_player()->get_max_db()) {
		att = get_audio_player()->get_max_db();
	}

	return att;
}

#ifndef PHYSICS_3D_DISABLED
void AudioSpatializerInstance3D::calc_reverb_vol(Area3D *area, Vector3 listener_area_pos, Vector<Vector2> direct_path_vol, Vector<Vector2> &reverb_vol) {
	reverb_vol.resize(MAX_CHANNELS_PER_BUS);
	reverb_vol.fill(Vector2(0, 0));

	float uniformity = area->get_reverb_uniformity();
	float area_send = area->get_reverb_amount();

	if (uniformity > 0.0) {
		float distance = listener_area_pos.length();
		float attenuation = Math::db_to_linear(get_attenuation_db(distance));

		// Determine the fraction of sound that would come from each speaker if they were all driven uniformly.
		float center_val[4] = { 0.5f, 0.25f, 0.16666f, 0.125f };
		int chan_count = AudioServer::get_singleton()->get_channel_count();
		AudioFrame center_frame(center_val[chan_count - 1], center_val[chan_count - 1]);

		if (attenuation < 1.0) {
			//pan the uniform sound
			Vector3 rev_pos = listener_area_pos;
			rev_pos.y = 0;
			rev_pos.normalize();

			calc_output_vol(rev_pos, reverb_vol);

			for (int i = 0; i < chan_count; i++) {
				reverb_vol.write[i] = reverb_vol[i].lerp(center_frame, attenuation);
			}
		} else {
			for (int i = 0; i < chan_count; i++) {
				reverb_vol.write[i] = center_frame;
			}
		}

		for (int i = 0; i < chan_count; i++) {
			reverb_vol.write[i] = direct_path_vol[i].lerp(reverb_vol[i] * attenuation, uniformity);
			reverb_vol.write[i] *= area_send;
		}

	} else {
		for (int i = 0; i < 4; i++) {
			reverb_vol.write[i] = direct_path_vol[i] * area_send;
		}
	}
}
#endif // PHYSICS_3D_DISABLED

Ref<SpatializerPlaybackData> AudioSpatializerInstance3D::instantiate_playback_data() {
	Ref<SpatializerPlaybackData3D> playback_data;
	playback_data.instantiate();
	return playback_data;
}

#ifndef PHYSICS_3D_DISABLED
// Interacts with PhysicsServer3D, so can only be called during _physics_process
Area3D *AudioSpatializerInstance3D::_get_overriding_area() {
	//check if any area is diverting sound into a bus
	Ref<World3D> world_3d = get_audio_player()->get_world_3d();
	ERR_FAIL_COND_V(world_3d.is_null(), nullptr);

	Vector3 global_pos = get_audio_player()->get_global_transform().origin;

	PhysicsDirectSpaceState3D *space_state = PhysicsServer3D::get_singleton()->space_get_direct_state(world_3d->get_space());

	PhysicsDirectSpaceState3D::ShapeResult sr[MAX_INTERSECT_AREAS];

	PhysicsDirectSpaceState3D::PointParameters point_params;
	point_params.position = global_pos;
	point_params.collision_mask = base->area_mask;
	point_params.collide_with_bodies = false;
	point_params.collide_with_areas = true;

	int areas = space_state->intersect_point(point_params, sr, MAX_INTERSECT_AREAS);

	for (int i = 0; i < areas; i++) {
		if (!sr[i].collider) {
			continue;
		}

		Area3D *tarea = Object::cast_to<Area3D>(sr[i].collider);
		if (!tarea) {
			continue;
		}

		if (!tarea->is_overriding_audio_bus() && !tarea->is_using_reverb_bus()) {
			continue;
		}

		return tarea;
	}
	return nullptr;
}
#endif // PHYSICS_3D_DISABLED

// Interacts with PhysicsServer3D, so can only be called during _physics_process.
StringName AudioSpatializerInstance3D::_get_actual_bus() {
#ifndef PHYSICS_3D_DISABLED
	Area3D *overriding_area = _get_overriding_area();
	if (overriding_area && overriding_area->is_overriding_audio_bus() && !overriding_area->is_using_reverb_bus()) {
		return overriding_area->get_audio_bus_name();
	}
#endif // PHYSICS_3D_DISABLED
	return get_audio_player()->get_bus();
}

static void _apply_max_volume(Vector<Vector2> &r_tgt_volume_vector, const Vector<Vector2> &p_src_volume_vector) {
	for (int64_t i = 0; i < r_tgt_volume_vector.size(); i++) {
		const Vector2 frame = Vector2(
				MAX(r_tgt_volume_vector[i][0], p_src_volume_vector[i][0]),
				MAX(r_tgt_volume_vector[i][1], p_src_volume_vector[i][1]));

		r_tgt_volume_vector.write[i] = frame;
	}
}

static float _get_max_volume(const Vector<Vector2> &p_src_volume) {
	float max_vol = 0.0;
	for (int64_t i = 0; i < p_src_volume.size(); i++) {
		max_vol = MAX(max_vol, p_src_volume[i][0]);
		max_vol = MAX(max_vol, p_src_volume[i][1]);
	}
	return max_vol;
}

Ref<SpatializerParameters> AudioSpatializerInstance3D::calculate_spatialization() {
	Ref<SpatializerParameters3D> parameters;

	static constexpr int64_t volume_vector_size = AudioServer::MAX_CHANNELS_PER_BUS;
	Vector3 global_pos = get_audio_player()->get_global_transform().origin;

	Ref<World3D> world_3d = get_audio_player()->get_world_3d();
	ERR_FAIL_COND_V(world_3d.is_null(), parameters);

	parameters.instantiate();

	HashSet<Camera3D *> cameras = world_3d->get_cameras();
	cameras.insert(get_audio_player()->get_viewport()->get_camera_3d());

#ifndef PHYSICS_3D_DISABLED
	PhysicsDirectSpaceState3D *space_state = PhysicsServer3D::get_singleton()->space_get_direct_state(world_3d->get_space());
	Area3D *area = _get_overriding_area();
#endif // PHYSICS_3D_DISABLED

	Vector3 linear_velocity;
	if (base->doppler_tracking != AudioSpatializer3D::DOPPLER_TRACKING_DISABLED) {
		linear_velocity = velocity_tracker->get_tracked_linear_velocity();
	}

	float log_pitch_scale = 0.0;
	float log_pitch_weight = 0.0;

	Vector<Vector2> output_volume;
	output_volume.resize(volume_vector_size);

	Vector<Vector2> reverb_volume;
	reverb_volume.resize(volume_vector_size);

	for (int i = 0; i < volume_vector_size; i++) {
		output_volume.write[i] = Vector2(0, 0);
		reverb_volume.write[i] = Vector2(0, 0);
	}

	Vector<Vector2> tmp_volume;
	tmp_volume.resize(volume_vector_size);

	Vector<Vector2> tmp_reverb;
	tmp_reverb.resize(volume_vector_size);

	bool has_any_listener_in_range = false;

	for (Camera3D *camera : cameras) {
		if (!camera) {
			continue;
		}
		Viewport *vp = camera->get_viewport();
		if (!vp) {
			continue;
		}
		if (!vp->is_audio_listener_3d()) {
			continue;
		}

		Node3D *listener_node = camera;

		AudioListener3D *listener = vp->get_audio_listener_3d();
		if (listener) {
			listener_node = listener;
		}

		const Vector3 local_pos = listener_node->get_global_transform().orthonormalized().affine_inverse().xform(global_pos);

		const float dist = local_pos.length();

#ifndef PHYSICS_3D_DISABLED
		Vector3 area_sound_pos;
		Vector3 listener_area_pos;

		if (area && area->is_using_reverb_bus() && area->get_reverb_uniformity() > 0) {
			area_sound_pos = space_state->get_closest_point_to_object_volume(area->get_rid(), listener_node->get_global_transform().origin);
			listener_area_pos = listener_node->get_global_transform().affine_inverse().xform(area_sound_pos);
		}
#endif // PHYSICS_3D_DISABLED

		// Vector<Vector2> output_volume_vector;
		// output_volume_vector.resize(AudioServer::MAX_CHANNELS_PER_BUS);

		float multiplier = Math::db_to_linear(get_attenuation_db(dist));

		if (base->max_distance > 0) {
			float total_max = base->max_distance;

#ifndef PHYSICS_3D_DISABLED
			if (area && area->is_using_reverb_bus() && area->get_reverb_uniformity() > 0) {
				total_max = MAX(total_max, listener_area_pos.length());
			}
#endif // PHYSICS_3D_DISABLED
			if (dist > total_max || total_max > base->max_distance) {
				continue; //can't hear this sound in this listener
			}
			multiplier *= MAX(0, 1.0 - (dist / base->max_distance));
		}
		has_any_listener_in_range = true;

		float db_att = (1.0 - MIN(1.0, multiplier)) * base->attenuation_filter_db;

		if (base->emission_angle_enabled) {
			Vector3 listenertopos = global_pos - listener_node->get_global_transform().origin;
			float c = listenertopos.normalized().dot(get_audio_player()->get_global_transform().basis.get_column(2).normalized()); //it's z negative
			float angle = Math::rad_to_deg(Math::acos(c));
			if (angle > base->emission_angle) {
				db_att -= -base->emission_angle_filter_attenuation_db;
			}
		}

		parameters->set_linear_attenuation(Math::db_to_linear(db_att));
		parameters->set_attenuation_filter_cutoff_hz(base->get_attenuation_filter_cutoff_hz());

		tmp_volume.fill(Vector2(0, 0));
		calc_output_vol(local_pos, tmp_volume);

		for (int64_t k = 0; k < tmp_volume.size(); k++) {
			tmp_volume.write[k] = multiplier * tmp_volume[k];
		}
		_apply_max_volume(output_volume, tmp_volume);

#ifndef PHYSICS_3D_DISABLED
		if (area && area->is_using_reverb_bus()) {
			calc_reverb_vol(area, listener_area_pos, tmp_volume, tmp_reverb);
			_apply_max_volume(reverb_volume, tmp_reverb);
		}
#endif

		if (base->doppler_tracking != AudioSpatializer3D::DOPPLER_TRACKING_DISABLED) {
			Vector3 listener_velocity;

			if (listener) {
				listener_velocity = listener->get_doppler_tracked_velocity();
			} else {
				listener_velocity = camera->get_doppler_tracked_velocity();
			}

			Vector3 local_velocity = listener_node->get_global_transform().orthonormalized().basis.xform_inv(linear_velocity - listener_velocity);

			if (local_velocity != Vector3()) {
				float approaching = local_pos.normalized().dot(local_velocity.normalized());
				float velocity = local_velocity.length();

				float doppler_pitch_scale = get_audio_player()->get_pitch_scale() * base->doppler_speed_of_sound / (base->doppler_speed_of_sound + velocity * approaching);
				doppler_pitch_scale = CLAMP(doppler_pitch_scale, (1 / 8.0), 8.0); //avoid crazy stuff

				float weight = _get_max_volume(tmp_volume);
				log_pitch_scale += weight * Math::log2(doppler_pitch_scale);
				log_pitch_weight += weight;
			}
		}
	}

	if (log_pitch_weight > 0) {
		parameters->set_pitch_scale(Math::pow(2.0f, log_pitch_scale / log_pitch_weight));
	} else {
		parameters->set_pitch_scale(get_audio_player()->get_pitch_scale());
	}

	//HashMap<StringName, Vector<Vector2>> bus_volumes;
	if (has_any_listener_in_range) {
#ifndef PHYSICS_3D_DISABLED
		if (area) {
			if (area->is_overriding_audio_bus()) {
				//override audio bus
				//bus_volumes[area->get_audio_bus_name()] = output_volume;
				parameters->add_bus_volume(area->get_audio_bus_name(), output_volume);
			} else {
				// I think this is supposed to direct to main bus if area override isn't enabled (GH-104382)
				parameters->add_bus_volume(get_audio_player()->get_bus(), output_volume);
			}

			if (area->is_using_reverb_bus()) {
				// StringName reverb_bus_name = area->get_reverb_bus_name();
				// bus_volumes[reverb_bus_name] = reverb_volume;
				parameters->add_bus_volume(area->get_reverb_bus_name(), reverb_volume);
			}
		} else
#endif // PHYSICS_3D_DISABLED
		{
			//bus_volumes[bus] = output_volume;
			//bus_volumes[get_audio_player()->get_bus()] = output_volume;
			parameters->add_bus_volume(get_audio_player()->get_bus(), output_volume);
		}
	}

	parameters->set_mix_volumes(output_volume);

	// if no listeners are in range and this was the case last frame, then we can skip setting any audio
	const bool skip_setting_volumes = !has_any_listener_in_range && was_further_than_max_distance_last_frame;
	was_further_than_max_distance_last_frame = !has_any_listener_in_range;

	if (!skip_setting_volumes) {
		// Want to update parameters that are sent to AudioServer, since they changed
		parameters->set_update_parameters(true);

		// for (Ref<AudioStreamPlayback> &playback : internal->stream_playbacks) {
		// 	AudioServer::get_singleton()->set_playback_bus_volumes_linear(playback, bus_volumes);
		// }

		// for (Ref<AudioStreamPlayback> &playback : internal->stream_playbacks) {
		// 	AudioServer::get_singleton()->set_playback_pitch_scale(playback, actual_pitch_scale);
		// 	if (playback->get_is_sample()) {
		// 		Ref<AudioSamplePlayback> sample_playback = playback->get_sample_playback();
		// 		if (sample_playback.is_valid()) {
		// 			AudioServer::get_singleton()->update_sample_playback_pitch_scale(sample_playback, actual_pitch_scale);
		// 		}
		// 	}
		// }
	}

	return parameters;
}

void AudioSpatializerInstance3D::process_frames(Ref<SpatializerParameters> p_parameters, Ref<SpatializerPlaybackData> p_playback_data,
		AudioFrame *p_output_buf, const AudioFrame *p_source_buf, int p_frame_count) {
	ERR_FAIL_COND_MSG(!Object::cast_to<SpatializerParameters3D>(*p_parameters), "Unexpected SpatializerParameters type; expected SpatializerParameters3D");
	ERR_FAIL_COND_MSG(!Object::cast_to<SpatializerPlaybackData3D>(*p_playback_data), "Unexpected SpatializerPlaybackData type; expected SpatializerPlaybackData3D");

	Ref<SpatializerParameters3D> parameters = Ref<SpatializerParameters3D>(Object::cast_to<SpatializerParameters3D>(*p_parameters));
	Ref<SpatializerPlaybackData3D> playback_data = Ref<SpatializerPlaybackData3D>(Object::cast_to<SpatializerPlaybackData3D>(*p_playback_data));

	Vector<Vector2> volumes = parameters->get_mix_volumes();
	AudioFrame p_prev_vol = AudioFrame(playback_data->get_prev_mix_volume(0));

	float highshelf_gain = parameters->get_linear_attenuation();
	if (highshelf_gain >= 0.001) {
		AudioFilterSW filter;
		filter.set_mode(AudioFilterSW::HIGHSHELF);
		filter.set_sampling_rate(AudioServer::get_singleton()->get_mix_rate());
		filter.set_cutoff(parameters->get_attenuation_filter_cutoff_hz());
		filter.set_resonance(1);
		filter.set_stages(1);
		filter.set_gain(highshelf_gain);

		AudioFilterSW::Processor *processor_l = playback_data->get_filter_processor(0, true);
		AudioFilterSW::Processor *processor_r = playback_data->get_filter_processor(0, false);

		ERR_FAIL_NULL(processor_l);
		ERR_FAIL_NULL(processor_r);

		bool is_just_started = p_prev_vol.left == 0 && p_prev_vol.right == 0;
		processor_l->set_filter(&filter, /* clear_history= */ is_just_started);
		processor_l->update_coeffs(p_frame_count);
		processor_r->set_filter(&filter, /* clear_history= */ is_just_started);
		processor_r->update_coeffs(p_frame_count);

		for (int frame_idx = 0; frame_idx < p_frame_count; frame_idx++) {
			AudioFrame mixed = p_source_buf[frame_idx];
			processor_l->process_one_interp(mixed.left);
			processor_r->process_one_interp(mixed.right);
			p_output_buf[frame_idx] = mixed;
		}
	} else {
		for (int frame_idx = 0; frame_idx < p_frame_count; frame_idx++) {
			AudioFrame mixed = p_source_buf[frame_idx];
			p_output_buf[frame_idx] = mixed;
		}
	}

	float max_volume = 0.0;
	int max_index = 0;
	for (int i = 0; i < MAX_CHANNELS_PER_BUS; i++) {
		if (volumes[i][0] > max_volume) {
			max_volume = volumes[i][0];
			max_index = i;
		}
		if (volumes[i][1] > max_volume) {
			max_volume = volumes[i][1];
			max_index = i;
		}
	}

	//print_verbose(vformat("AudioSpatializerInstance3D mix_channel updating prev volume"));
	playback_data->set_prev_mix_volume(0, volumes[max_index]);
}

void AudioSpatializerInstance3D::mix_channel(Ref<SpatializerParameters> p_parameters, Ref<SpatializerPlaybackData> p_playback_data, int p_channel, AudioFrame *p_out_buf, const AudioFrame *p_source_buf, int p_frame_count) {
	// Mix p_frame_count audio frames from an audio source p_source_buf, for channel p_channel, using spatial_parameters calculated from the audio player of the source.
	ERR_FAIL_COND_MSG(!Object::cast_to<SpatializerParameters3D>(*p_parameters), "Unexpected SpatializerParameters type; expected SpatializerParameters3D");
	ERR_FAIL_COND_MSG(!Object::cast_to<SpatializerPlaybackData3D>(*p_playback_data), "Unexpected SpatializerPlaybackData type; expected SpatializerPlaybackData3D");

	Ref<SpatializerParameters3D> parameters = Ref<SpatializerParameters3D>(Object::cast_to<SpatializerParameters3D>(*p_parameters));
	Ref<SpatializerPlaybackData3D> playback_data = Ref<SpatializerPlaybackData3D>(Object::cast_to<SpatializerPlaybackData3D>(*p_playback_data));

	Vector<Vector2> volumes = parameters->get_mix_volumes();

	AudioFrame p_vol_start = AudioFrame(playback_data->get_prev_mix_volume(p_channel));
	AudioFrame p_vol_final = AudioFrame(volumes[p_channel]);

	float highshelf_gain = parameters->get_linear_attenuation();
	if (highshelf_gain >= 0.001) {
		AudioFilterSW filter;
		filter.set_mode(AudioFilterSW::HIGHSHELF);
		filter.set_sampling_rate(AudioServer::get_singleton()->get_mix_rate());
		filter.set_cutoff(parameters->get_attenuation_filter_cutoff_hz());
		filter.set_resonance(1);
		filter.set_stages(1);
		filter.set_gain(highshelf_gain);

		AudioFilterSW::Processor *processor_l = playback_data->get_filter_processor(p_channel, true);
		AudioFilterSW::Processor *processor_r = playback_data->get_filter_processor(p_channel, false);

		ERR_FAIL_NULL(processor_l);
		ERR_FAIL_NULL(processor_r);

		bool is_just_started = p_vol_start.left == 0 && p_vol_start.right == 0;
		processor_l->set_filter(&filter, /* clear_history= */ is_just_started);
		processor_l->update_coeffs(p_frame_count);
		processor_r->set_filter(&filter, /* clear_history= */ is_just_started);
		processor_r->update_coeffs(p_frame_count);

		for (int frame_idx = 0; frame_idx < p_frame_count; frame_idx++) {
			// TODO: Make lerp speed buffer-size-invariant if buffer_size ever becomes a project setting to avoid very small buffer sizes causing pops due to too-fast lerps.
			float lerp_param = (float)frame_idx / p_frame_count;
			AudioFrame vol = p_vol_final * lerp_param + (1 - lerp_param) * p_vol_start;
			AudioFrame mixed = vol * p_source_buf[frame_idx];
			processor_l->process_one_interp(mixed.left);
			processor_r->process_one_interp(mixed.right);
			p_out_buf[frame_idx] = mixed;
		}

	} else {
		for (int frame_idx = 0; frame_idx < p_frame_count; frame_idx++) {
			// TODO: Make lerp speed buffer-size-invariant if buffer_size ever becomes a project setting to avoid very small buffer sizes causing pops due to too-fast lerps.
			float lerp_param = (float)frame_idx / p_frame_count;
			p_out_buf[frame_idx] = (p_vol_final * lerp_param + (1 - lerp_param) * p_vol_start) * p_source_buf[frame_idx];
		}
	}

	//print_verbose(vformat("AudioSpatializerInstance3D mix_channel updating prev volume"));
	playback_data->set_prev_mix_volume(p_channel, volumes[p_channel]);
}

void AudioSpatializerInstance3D::initialize_audio_player() {
	if (get_audio_player() == nullptr) {
		return;
	}

	if (base->doppler_tracking != AudioSpatializer3D::DOPPLER_TRACKING_DISABLED) {
		get_audio_player()->add_transform_changed_callback(_transform_changed_cb, this);
		velocity_tracker->set_track_physics_step(base->doppler_tracking == AudioSpatializer3D::DOPPLER_TRACKING_PHYSICS_STEP);
		if (get_audio_player()->is_inside_tree()) {
			velocity_tracker->reset(get_audio_player()->get_global_transform().origin);
		}
	}
}

void AudioSpatializerInstance3D::update_doppler_tracked_velocity() {
	if (base->doppler_tracking != AudioSpatializer3D::DOPPLER_TRACKING_DISABLED) {
		velocity_tracker->update_position(get_audio_player()->get_global_transform().origin);
	}
}

AudioSpatializerInstance3D::AudioSpatializerInstance3D() {
	velocity_tracker.instantiate();
	cached_global_panning_strength = GLOBAL_GET_CACHED(float, "audio/general/3d_panning_strength");
}

AudioSpatializerInstance3D::~AudioSpatializerInstance3D() {
	if (get_audio_player() != nullptr) {
		get_audio_player()->remove_transform_changed_callback(_transform_changed_cb, this);
	}
	velocity_tracker.unref();
}

/////////////////////////////////////////////////////////////////////////////////////////

Ref<AudioSpatializerInstance> AudioSpatializer3D::instantiate() {
	Ref<AudioSpatializerInstance3D> ins;
	ins.instantiate();
	ins->base = Ref<AudioSpatializer3D>(this);
	ins->mix_channel_mode = mix_channel_mode;
	//ins->init_channels_and_buffers();
	return ins;
}

void AudioSpatializer3D::set_mix_channel_mode(bool p_mode) {
	mix_channel_mode = p_mode;
}

bool AudioSpatializer3D::get_mix_channel_mode() const {
	return mix_channel_mode;
}

void AudioSpatializer3D::set_unit_size(float p_volume) {
	unit_size = p_volume;
}

float AudioSpatializer3D::get_unit_size() const {
	return unit_size;
}

void AudioSpatializer3D::set_max_distance(float p_metres) {
	ERR_FAIL_COND(p_metres < 0.0);
	max_distance = p_metres;
}

float AudioSpatializer3D::get_max_distance() const {
	return max_distance;
}

void AudioSpatializer3D::set_area_mask(uint32_t p_mask) {
	area_mask = p_mask;
}

uint32_t AudioSpatializer3D::get_area_mask() const {
	return area_mask;
}

void AudioSpatializer3D::set_emission_angle_enabled(bool p_enable) {
	emission_angle_enabled = p_enable;
}

bool AudioSpatializer3D::is_emission_angle_enabled() const {
	return emission_angle_enabled;
}

void AudioSpatializer3D::set_emission_angle(float p_angle) {
	ERR_FAIL_COND(p_angle < 0 || p_angle > 90);
	emission_angle = p_angle;
}

float AudioSpatializer3D::get_emission_angle() const {
	return emission_angle;
}

void AudioSpatializer3D::set_emission_angle_filter_attenuation_db(float p_angle_attenuation_db) {
	emission_angle_filter_attenuation_db = p_angle_attenuation_db;
}

float AudioSpatializer3D::get_emission_angle_filter_attenuation_db() const {
	return emission_angle_filter_attenuation_db;
}

void AudioSpatializer3D::set_attenuation_filter_cutoff_hz(float p_hz) {
	attenuation_filter_cutoff_hz = p_hz;
}

float AudioSpatializer3D::get_attenuation_filter_cutoff_hz() const {
	return attenuation_filter_cutoff_hz;
}

void AudioSpatializer3D::set_attenuation_filter_db(float p_db) {
	attenuation_filter_db = p_db;
}

float AudioSpatializer3D::get_attenuation_filter_db() const {
	return attenuation_filter_db;
}

void AudioSpatializer3D::set_attenuation_model(AttenuationModel p_model) {
	ERR_FAIL_INDEX((int)p_model, 4);
	attenuation_model = p_model;
}

AudioSpatializer3D::AttenuationModel AudioSpatializer3D::get_attenuation_model() const {
	return attenuation_model;
}

void AudioSpatializer3D::set_panning_strength(float p_panning_strength) {
	ERR_FAIL_COND_MSG(p_panning_strength < 0, "Panning strength must be a positive number.");
	panning_strength = p_panning_strength;
}

float AudioSpatializer3D::get_panning_strength() const {
	return panning_strength;
}

void AudioSpatializer3D::set_doppler_tracking(DopplerTracking p_tracking) {
	if (doppler_tracking == p_tracking) {
		return;
	}

	doppler_tracking = p_tracking;
}

AudioSpatializer3D::DopplerTracking AudioSpatializer3D::get_doppler_tracking() const {
	return doppler_tracking;
}

void AudioSpatializer3D::set_doppler_speed_of_sound(float p_speed) {
	ERR_FAIL_COND_MSG(p_speed <= 0, "Speed of sound must be a positive number.");
	doppler_speed_of_sound = p_speed;
}

float AudioSpatializer3D::get_doppler_speed_of_sound() const {
	return doppler_speed_of_sound;
}

void AudioSpatializer3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_mix_channel_mode", "mode"), &AudioSpatializer3D::set_mix_channel_mode);
	ClassDB::bind_method(D_METHOD("get_mix_channel_mode"), &AudioSpatializer3D::get_mix_channel_mode);

	ClassDB::bind_method(D_METHOD("set_unit_size", "unit_size"), &AudioSpatializer3D::set_unit_size);
	ClassDB::bind_method(D_METHOD("get_unit_size"), &AudioSpatializer3D::get_unit_size);

	ClassDB::bind_method(D_METHOD("set_max_distance", "meters"), &AudioSpatializer3D::set_max_distance);
	ClassDB::bind_method(D_METHOD("get_max_distance"), &AudioSpatializer3D::get_max_distance);

	ClassDB::bind_method(D_METHOD("set_panning_strength", "panning_strength"), &AudioSpatializer3D::set_panning_strength);
	ClassDB::bind_method(D_METHOD("get_panning_strength"), &AudioSpatializer3D::get_panning_strength);

	ClassDB::bind_method(D_METHOD("set_area_mask", "mask"), &AudioSpatializer3D::set_area_mask);
	ClassDB::bind_method(D_METHOD("get_area_mask"), &AudioSpatializer3D::get_area_mask);

	ClassDB::bind_method(D_METHOD("set_emission_angle", "degrees"), &AudioSpatializer3D::set_emission_angle);
	ClassDB::bind_method(D_METHOD("get_emission_angle"), &AudioSpatializer3D::get_emission_angle);

	ClassDB::bind_method(D_METHOD("set_emission_angle_enabled", "enabled"), &AudioSpatializer3D::set_emission_angle_enabled);
	ClassDB::bind_method(D_METHOD("is_emission_angle_enabled"), &AudioSpatializer3D::is_emission_angle_enabled);

	ClassDB::bind_method(D_METHOD("set_emission_angle_filter_attenuation_db", "db"), &AudioSpatializer3D::set_emission_angle_filter_attenuation_db);
	ClassDB::bind_method(D_METHOD("get_emission_angle_filter_attenuation_db"), &AudioSpatializer3D::get_emission_angle_filter_attenuation_db);

	ClassDB::bind_method(D_METHOD("set_attenuation_filter_cutoff_hz", "degrees"), &AudioSpatializer3D::set_attenuation_filter_cutoff_hz);
	ClassDB::bind_method(D_METHOD("get_attenuation_filter_cutoff_hz"), &AudioSpatializer3D::get_attenuation_filter_cutoff_hz);

	ClassDB::bind_method(D_METHOD("set_attenuation_filter_db", "db"), &AudioSpatializer3D::set_attenuation_filter_db);
	ClassDB::bind_method(D_METHOD("get_attenuation_filter_db"), &AudioSpatializer3D::get_attenuation_filter_db);

	ClassDB::bind_method(D_METHOD("set_attenuation_model", "model"), &AudioSpatializer3D::set_attenuation_model);
	ClassDB::bind_method(D_METHOD("get_attenuation_model"), &AudioSpatializer3D::get_attenuation_model);

	ClassDB::bind_method(D_METHOD("set_doppler_tracking", "mode"), &AudioSpatializer3D::set_doppler_tracking);
	ClassDB::bind_method(D_METHOD("get_doppler_tracking"), &AudioSpatializer3D::get_doppler_tracking);

	ClassDB::bind_method(D_METHOD("set_doppler_speed_of_sound", "speed"), &AudioSpatializer3D::set_doppler_speed_of_sound);
	ClassDB::bind_method(D_METHOD("get_doppler_speed_of_sound"), &AudioSpatializer3D::get_doppler_speed_of_sound);

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "mix_channel_mode"), "set_mix_channel_mode", "get_mix_channel_mode");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "attenuation_model", PROPERTY_HINT_ENUM, "Inverse,Inverse Square,Logarithmic,Disabled"), "set_attenuation_model", "get_attenuation_model");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "unit_size", PROPERTY_HINT_RANGE, "0.1,100,0.01,or_greater"), "set_unit_size", "get_unit_size");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_distance", PROPERTY_HINT_RANGE, "0,4096,0.01,or_greater,suffix:m"), "set_max_distance", "get_max_distance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "panning_strength", PROPERTY_HINT_RANGE, "0,3,0.01,or_greater"), "set_panning_strength", "get_panning_strength");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "area_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_area_mask", "get_area_mask");

	ADD_GROUP("Emission Angle", "emission_angle");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "emission_angle_enabled", PROPERTY_HINT_GROUP_ENABLE), "set_emission_angle_enabled", "is_emission_angle_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "emission_angle_degrees", PROPERTY_HINT_RANGE, "0.1,90,0.1,degrees"), "set_emission_angle", "get_emission_angle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "emission_angle_filter_attenuation_db", PROPERTY_HINT_RANGE, "-80,0,0.1,suffix:dB"), "set_emission_angle_filter_attenuation_db", "get_emission_angle_filter_attenuation_db");

	ADD_GROUP("Attenuation Filter", "attenuation_filter_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "attenuation_filter_cutoff_hz", PROPERTY_HINT_RANGE, "1,20500,1,suffix:Hz"), "set_attenuation_filter_cutoff_hz", "get_attenuation_filter_cutoff_hz");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "attenuation_filter_db", PROPERTY_HINT_RANGE, "-80,0,0.1,suffix:dB"), "set_attenuation_filter_db", "get_attenuation_filter_db");

	ADD_GROUP("Doppler", "doppler_");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "doppler_tracking", PROPERTY_HINT_ENUM, "Disabled,Idle,Physics"), "set_doppler_tracking", "get_doppler_tracking");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "doppler_speed_of_sound", PROPERTY_HINT_RANGE, "1.0,3000,0.1,suffix:m/s"), "set_doppler_speed_of_sound", "get_doppler_speed_of_sound");

	BIND_ENUM_CONSTANT(ATTENUATION_INVERSE_DISTANCE);
	BIND_ENUM_CONSTANT(ATTENUATION_INVERSE_SQUARE_DISTANCE);
	BIND_ENUM_CONSTANT(ATTENUATION_LOGARITHMIC);
	BIND_ENUM_CONSTANT(ATTENUATION_DISABLED);

	BIND_ENUM_CONSTANT(DOPPLER_TRACKING_DISABLED);
	BIND_ENUM_CONSTANT(DOPPLER_TRACKING_IDLE_STEP);
	BIND_ENUM_CONSTANT(DOPPLER_TRACKING_PHYSICS_STEP);
}

AudioSpatializer3D::AudioSpatializer3D() {
	spcap = memnew(SpeakerPlacementConfiguration);
}

AudioSpatializer3D::~AudioSpatializer3D() {
	memdelete(spcap);
}

/////////////////////////////////////////////////////////////////////////////////////////

void SpatializerParameters3D::set_linear_attenuation(float p_att) {
	linear_attenuation = p_att;
}

float SpatializerParameters3D::get_linear_attenuation() const {
	return linear_attenuation;
}

void SpatializerParameters3D::set_attenuation_filter_cutoff_hz(float p_cutoff) {
	attenuation_filter_cutoff_hz = p_cutoff;
}

float SpatializerParameters3D::get_attenuation_filter_cutoff_hz() const {
	return attenuation_filter_cutoff_hz;
}

void SpatializerParameters3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_linear_attenuation", "att"), &SpatializerParameters3D::set_linear_attenuation);
	ClassDB::bind_method(D_METHOD("get_linear_attenuation"), &SpatializerParameters3D::get_linear_attenuation);

	ClassDB::bind_method(D_METHOD("set_attenuation_filter_cutoff_hz", "pitch_scale"), &SpatializerParameters3D::set_attenuation_filter_cutoff_hz);
	ClassDB::bind_method(D_METHOD("get_attenuation_filter_cutoff_hz"), &SpatializerParameters3D::get_attenuation_filter_cutoff_hz);
}

///////////////////////////////////////////////////////////////////////////////

void SpatializerPlaybackData3D::set_prev_mix_volume(int p_channel, Vector2 p_volume) {
	if (prev_mix_volumes.size() <= p_channel) {
		prev_mix_volumes.resize(p_channel + 1);
	}
	prev_mix_volumes.write[p_channel] = p_volume;
}

Vector2 SpatializerPlaybackData3D::get_prev_mix_volume(int p_channel) const {
	if (prev_mix_volumes.size() <= p_channel) {
		return Vector2(0.0, 0.0);
	}
	return prev_mix_volumes[p_channel];
}

AudioFilterSW::Processor *SpatializerPlaybackData3D::get_filter_processor(int p_channel, bool left) {
	ERR_FAIL_INDEX_V(p_channel, 4, nullptr);
	if (left) {
		return &filter_processors[p_channel * 2];
	} else {
		return &filter_processors[p_channel * 2 + 1];
	}
}

void SpatializerPlaybackData3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_prev_mix_volume", "channel", "volume"), &SpatializerPlaybackData3D::set_prev_mix_volume);
	ClassDB::bind_method(D_METHOD("get_prev_mix_volume", "channel"), &SpatializerPlaybackData3D::get_prev_mix_volume);
}

///////////////////////////////////////////////////////////////////////////////

void SpeakerPlacementConfiguration::update_speaker_configuration(unsigned int speaker_count, const Vector3 *directions) {
	speakers.resize(speaker_count);
	Speaker *w = speakers.ptrw();
	for (unsigned int speaker_num = 0; speaker_num < speaker_count; speaker_num++) {
		w[speaker_num].direction = directions[speaker_num];
		w[speaker_num].squared_gain = 0.0;
		w[speaker_num].effective_number_of_speakers = 0.0;
	}
	for (unsigned int speaker_num = 0; speaker_num < speaker_count; speaker_num++) {
		for (unsigned int other_speaker_num = 0; other_speaker_num < speaker_count; other_speaker_num++) {
			w[speaker_num].effective_number_of_speakers += 0.5 * (1.0 + w[speaker_num].direction.dot(w[other_speaker_num].direction));
		}
	}
}

unsigned int SpeakerPlacementConfiguration::get_speaker_count() const {
	return (unsigned int)speakers.size();
}

Vector3 SpeakerPlacementConfiguration::get_speaker_direction(unsigned int index) const {
	return speakers.ptr()[index].direction;
}

void SpeakerPlacementConfiguration::calculate(const Vector3 &source_direction, real_t tightness, unsigned int volume_count, real_t *volumes) const {
	const Speaker *r = speakers.ptr();
	real_t sum_squared_gains = 0.0;
	for (unsigned int speaker_num = 0; speaker_num < (unsigned int)speakers.size(); speaker_num++) {
		real_t initial_gain = 0.5 * std::pow(1.0 + r[speaker_num].direction.dot(source_direction), tightness) / r[speaker_num].effective_number_of_speakers;
		r[speaker_num].squared_gain = initial_gain * initial_gain;
		sum_squared_gains += r[speaker_num].squared_gain;
	}

	for (unsigned int speaker_num = 0; speaker_num < MIN(volume_count, (unsigned int)speakers.size()); speaker_num++) {
		volumes[speaker_num] = std::sqrt(r[speaker_num].squared_gain / sum_squared_gains);
	}
}
