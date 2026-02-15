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

#include "servers/audio/audio_stream.h"

#include "audio_stream_player_spatial.h"
#include "spatializer_parameters.h"

class AudioSpatializer;
class AudioStreamPlaybackSpatial;

class AudioSpatializerInstance : public RefCounted {
	GDCLASS(AudioSpatializerInstance, RefCounted);
	friend class AudioStreamPlayerSpatial;
	friend class AudioStreamPlaybackSpatial;

public:
	enum {
		MAX_CHANNELS_PER_BUS = 4, // Should match AudioServer::MAX_CHANNELS_PER_BUS
		LOOKAHEAD_BUFFER_SIZE = 64,
		MAX_BUSES_PER_PLAYBACK = 6,
		MAX_INTERSECT_AREAS = 32,
	};

private:
	struct SpatialPlaybackListNode {
		// The state machine for audio stream playbacks is as follows:
		// 1. The playback is created and added to the playback list in the playing state.
		// 2. The playback is (maybe) paused, and the state is set to FADE_OUT_TO_PAUSE.
		// 2.1. The playback is mixed after being paused, and the audio server thread atomically sets the state to PAUSED after performing a brief fade-out.
		// 3. The playback is (maybe) deleted, and the state is set to FADE_OUT_TO_DELETION.
		// 3.1. The playback is mixed after being deleted, and the audio server thread atomically sets the state to AWAITING_DELETION after performing a brief fade-out.
		// 		NOTE: The playback is not deallocated at this time because allocation and deallocation are not realtime-safe.
		// 4. The playback is removed and deallocated on the main thread using the SafeList maybe_cleanup method.
		// enum PlaybackState {
		// 	PAUSED = 0, // Paused. Keep this stream playback around though so it can be restarted.
		// 	PLAYING = 1, // Playing. Fading may still be necessary if volume changes!
		// 	FADE_OUT_TO_PAUSE = 2, // About to pause.
		// 	FADE_OUT_TO_DELETION = 3, // About to stop.
		// 	AWAITING_DELETION = 4,
		// };
		// Updating this ref after the list node is created also breaks consistency guarantees, don't do it!
		Ref<AudioStreamPlayback> stream_playback;
		// Persistant playback data maintained between frames. Modified by mix_channel.
		Ref<SpatializerPlaybackData> playback_data;
		// Playback state determines the fate of a particular AudioStreamListNode during the mix step. Must be atomically replaced.
		// std::atomic<PlaybackState> state;
		SafeFlag active;
		// The next few samples are stored here so we have some time to fade audio out if it ends abruptly at the beginning of the next mix.
		AudioFrame lookahead[LOOKAHEAD_BUFFER_SIZE];
	};

	uint64_t mixed_frame_count = 0;

	SafeList<SpatialPlaybackListNode *> playback_list;

	// Stores per-channel stream playbacks that are passed to AudioServer; should be up to 4, one per channel (depending on AudioServer::get_channel_count()).
	Vector<Ref<AudioStreamPlaybackSpatial>> spatial_playbacks;

	AudioStreamPlayerSpatial *audio_player = nullptr;
	Ref<SpatializerParameters> spatializer_parameters;

	SafeFlag playback_active;

	Vector<AudioFrame> playback_buffer;
	Vector<AudioFrame> process_buffer;
	Vector<Vector<AudioFrame>> mix_buffer;
	SafeNumeric<int> channel_count;
	bool channel_mixed[MAX_CHANNELS_PER_BUS];

	Mutex mutex;

	SpatialPlaybackListNode *_find_playback_list_node(Ref<AudioStreamPlayback> p_playback);

	void _delete_stream_playback_list_node(SpatialPlaybackListNode *p_playback_node);

	void _mix_from_playback_list(int p_buffer_size);
	void _manage_playback_state();

	void _manage_player_list_state();
	bool _check_channel_mixed(int channel);

	void set_audio_player(AudioStreamPlayerSpatial *p_audio_player);
	void set_spatializer_parameters(Ref<SpatializerParameters> p_parameters);

protected:
	GDVIRTUAL0R_REQUIRED(Ref<SpatializerParameters>, _calculate_spatialization)
	GDVIRTUAL0R(Ref<SpatializerPlaybackData>, _instantiate_playback_data)

	GDVIRTUAL0RC(bool, _should_process_frames)
	GDVIRTUAL5(_process_frames, Ref<SpatializerParameters>, Ref<SpatializerPlaybackData>, GDExtensionPtr<AudioFrame>, GDExtensionConstPtr<AudioFrame>, int)

	GDVIRTUAL0RC(bool, _should_mix_channels)
	GDVIRTUAL6(_mix_channel, Ref<SpatializerParameters>, Ref<SpatializerPlaybackData>, int, GDExtensionPtr<AudioFrame>, GDExtensionConstPtr<AudioFrame>, int)

	GDVIRTUAL0(_initialize_audio_player)

	static void _bind_methods();

	void init_channels_and_buffers();

public:
	AudioSpatializerInstance();

	AudioStreamPlayerSpatial *get_audio_player() const;
	Ref<SpatializerParameters> get_spatializer_parameters() const;

	//void start_playback_stream(Ref<AudioStreamPlayback> p_playback, const HashMap<StringName, Vector<AudioFrame>> &p_bus_volumes, float p_start_time);
	void start_playback_stream(Ref<AudioStreamPlayback> p_playback, float p_start_time);
	void stop_playback_stream(Ref<AudioStreamPlayback> p_playback);
	void set_playback_paused(bool p_paused);
	bool is_playback_active(Ref<AudioStreamPlayback> p_playback);
	float get_playback_position(Ref<AudioStreamPlayback> p_playback);
	bool is_playback_paused();

	// Called from Audio Thread: AudioStreamSpatialPlayback.mix()
	void get_mixed_frames(int p_channel, AudioFrame *p_frames, int p_frame_count);

	// Called from Physics Thread: AudioStreamPlayerSpatial._notification(NOTIFICATION_INTERNAL_PHYSICS_PROCESS)
	void update_spatializer_parameters();

	// Get the bus map from the current spatializer parameters
	HashMap<StringName, Vector<AudioFrame>> get_bus_map(const Ref<SpatializerParameters> p_params, int p_channel);

	virtual Ref<SpatializerParameters> calculate_spatialization();
	virtual bool should_process_frames() const;
	virtual void process_frames(Ref<SpatializerParameters> p_parameters, Ref<SpatializerPlaybackData> p_playback_data, AudioFrame *p_output_buf, const AudioFrame *p_source_buf, int p_frame_count);
	virtual bool should_mix_channels() const;
	virtual void mix_channel(Ref<SpatializerParameters> p_parameters, Ref<SpatializerPlaybackData> p_playback_data, int p_channel, AudioFrame *p_output_buf, const AudioFrame *p_source_buf, int p_frame_count);
	virtual Ref<SpatializerPlaybackData> instantiate_playback_data();
	virtual void initialize_audio_player();
};

class AudioSpatializer : public Resource {
	GDCLASS(AudioSpatializer, Resource);

protected:
	GDVIRTUAL0R_REQUIRED(Ref<AudioSpatializerInstance>, _instantiate)
	static void _bind_methods();

public:
	virtual Ref<AudioSpatializerInstance> instantiate();
	AudioSpatializer();
};

class AudioStreamPlaybackSpatial : public AudioStreamPlayback {
	GDCLASS(AudioStreamPlaybackSpatial, AudioStreamPlayback);
	friend class AudioSpatializerInstance;

	SafeFlag active;
	int channel;
	AudioSpatializerInstance *spatializer;

protected:
	// static void _bind_methods();

public:
	virtual void start(double p_from_pos = 0.0) override;
	virtual void stop() override;
	virtual bool is_playing() const override;

	virtual int get_loop_count() const override; //times it looped

	virtual double get_playback_position() const override;
	virtual void tag_used_streams() override;

	virtual int mix(AudioFrame *p_buffer, float p_rate_scale, int p_frames) override;
};
