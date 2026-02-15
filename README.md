# Godot Audio Spatializer
**Godot Audio Spatializer** is a Godot Engine [custom C++ module](https://docs.godotengine.org/en/stable/engine_details/engine_api/custom_modules_in_cpp.html) that provides a framework for adding alternate audio spatialization calculations.

The base Godot Engine audio system handles 3D audio mixing in `AudioStreamPlayer3D` and the `AudioServer`, before processing it through the audio buses. `AudioStreamPlayer3D` does not expose, or allow for extending the mixing calculations. The **Godot Audio Spatializer** provides an alternate `AudioStreamPlayerSpatial` node that takes as a parameter an `AudioSpatializer` resource, which handles the 3D mixing, and can be extended to do custom spatialization via GDScript, C#, and GDExtension bindings.

Currently there are two implementations included with the module, `AudioSpatializer3D` and `AudioSpatializerEffect`. `AudioSpatializer3D` replicates the behavior of the `AudioStreamPlayer3D`, mostly as a proof of concept. It can be used as a reference when developing GDExtension based implementations. The flag `mix_channel_mode` shows how the different methods to override in `AudioSpatializer` differ in how they should be implemented.

`AudioSpatializerEffect` is a partial implementation that needs to be extended to be used. It includes the ability to add `AudioEffect`s to sounds coming from the `AudioSpatializer`, before it gets mixed into the target audio bus(es). It handles the lowest level audio processing in fast C++ code, while allowing for GDScript/C# extensions that do the spatialization calculation and audio bus selection.

## Building

To use the **Godot Audio Spatializer**, you need to build the Godot Engine from source, and include this module as a custom module. Godot has extensive documentation for [compiling the engine](https://docs.godotengine.org/en/stable/engine_details/development/compiling/index.html) from source. Including this module is as easy as adding it as a custom module ([compiling a module externally](https://docs.godotengine.org/en/stable/engine_details/engine_api/custom_modules_in_cpp.html#compiling-a-module-externally)) or adding it directly to the `modules/` path of the `godot` source tree. **Note:** The name of the directory that holds this module should be `audio_spatializer`, not the default name of this repository. The Godot module registration code uses reflection on the module path to find the initialize/uninitialize functions.

## Extending

There are four main classes that can be extended to implement a custom `AudioSpatializer`.

### AudioSpatializer

Handles the high level configuration of the spatializer. This is a resource, and as such, can be saved and shared between different `AudioStreamPlayerSpatial` nodes. Any configuration parameters that don't change at runtime can be stored here. The one method it has that requires overriding is `_instantiate`. This should create and configure your custom `AudioSpatializerInstance`, which is where the bulk of the logic will live.

### AudioSpatializerInstance

Every `AudioStreamPlayerSpatial` that has a specific `AudioSpatializer` will get an instance created for it. There are a number of methods to implement, some are required, many are optional.

* `_calculate_spatialization` [Required]: Calculates all relevant details required to mix sound into the audio system. These are stored in a `SpatializerParameters` object (which can be extended), which is returned by the method. This method is called on the Physics thread, and so any physics calculations (e.g. raycasts, intersections, etc) can be done here, and their results stored for later use. The base `SpatializerParameters` has a number of fields that should be populated, see below. Access to the parent `AudioStreamPlayerSpatial` is available via `get_audio_player()` to get sound specific properties (e.g. position, volume, pitch).

* `_instantiate_playback_data`: Returns a new `SpatializerPlaybackData` object each time a new playback is created (i.e. every time the sound plays). Override this method to return a custom playback data structure (along with any initialization needed for it) for your spatializer. The data structure is stored with the playback, and is passed into `_process_frames` and `_mix_channel` to be used for storing persistent data between calls to allow for temporal filtering. It can be read from and written to in either method safely.

* `_initialize_audio_player`: This method is called as part of instance creation, just after the parent `audio_player` reference is assigned. Useful for registering for callback or notifications from the `audio_player`, or setting default values based on an `audio_player` properties.

* `_process_frames`: An intermediate processing step on the single channel* audio stream before mixing. This is called from the `AudioServer` thread. _Note: this should only be implemented in a GDExtension; GDScript and C# don't work._ Parameters are:
    * `spatializer_parameters`: The most recently calculated parameters returned from `_calculate_spatialization`. These can be used to configure filters or process the audio stream frames.
    * `playback_data`: A data structure that is created per playback, and is persisted between frames to allow for temporal filtering.
    * `output_buf`: The output buffer to write the processed audio stream into.
    * `source_buf`: The input buffer to read the incoming audio stream info from.
    * `frame_count`: The length of `source_buf` and `output_buf`; i.e. the number of `AudioFrame`s to process.

* `_should_process_frames`: Returns a boolean telling the `AudioSpatializer` whether or not the call `_process_frames`. If no additional processing is required for your spatializer, this skips the buffer copy.

* `_mix_channel`: If your spatialization requires non-linear effects happening on per-channel basis (i.e. something that could not be achieved by using per-channel volumes modulating a single source audio stream), this method can be used to do custom mixing per channel.  This is called from the `AudioServer` thread. _Note: this should only be implemented in a GDExtension; GDScript and C# don't work._ Parameters are similar to `_process_frames`:
    * `spatializer_parameters`: The most recently calculated parameters returned from `_calculate_spatialization`. These can be used to configure filters or process the audio stream frames.
    * `playback_data`: A data structure that is created per playback, and is persisted between frames to allow for temporal filtering. To get per-channel data, add 4 parameters here, and use `channel` to select the it.
    * `channel`: The current audio channel to mix (**note:** this is a _stereo_ audio channel, which corresponds to one of four channel pairs the `AudioServer` uses internally; 0 is front left/right, 1 is center/LFE, 2 is rear left/right, 3 is side left/right).
    * `output_buf`: The output buffer to write the processed audio stream into, for the given `channel`.
    * `source_buf`: The input buffer to read the incoming audio stream info from.
    * `frame_count`: The length of `source_buf` and `output_buf`; i.e. the number of `AudioFrame`s to process.

* `_should_mix_channels`: Returns a boolean telling the `AudioSpatialzier` whether or not to call `_mix_channel`. It also changes how spatializer interacts with the `AudioServer`, and should only be used if you need access to the way frames are mixed into the stream. See Implementation Details below.

**Note:** you should not use member variables on the `AudioSpatializerInstance` to share data between `_calculate_spatializion`, `_process_frames`, and `_mix_channel`. These methods are running on different threads, and doing so can result in race conditions and generally undefined behavior. Add any important data to `SpatializerParameters`, and use them in the process/mix methods.

### SpatializerParameters

A simple data class to hold calculation results and parameters that are needed for spatializer processing. It is constructed and returned by `AudioSpatializerInstance` `_calculate_spatialiation`. It can be extended to store whatever info you need, but the built in properties should be used for their intended purpose.

* `mix_volumes`: `PackedVector2Array` of size 4, where each entry corresponds to an internal stereo audio channel (0 is front left/right, 1 is center/LFE, 2 is rear left/right, 3 is side left/right).
* `pitch_scale`: Pitch scale used to sample the internal `AudioStreamPlayback`s. Default should be `get_audio_player().pitch_scale`.
* `update_parameters`: Flag to tell the `AudioSpatializerInstance` whether or not to send the latest Audio bus map to the `AudioServer`. Is intended to avoid sending redundant data over and over, specifically when a sound is out of range of any listeners.
* `bus_volumes`: Not exposed as a property, but can be added to using `add_bus_volumes()` which takes a bus name, and a `PackedVector2Array` of size 4 which corresponds to the volume on that audio bus.

### SpatializerPlaybackData

A simple data class that can be extended to provide per-playback data, that is persisted between frames, and exists for the lifetime of the internal playback. It is only accessed from the same thread, and reading/writing to it is safe. It is constructed and returned by `AudioSpatializerInstance` `_instantiate_playback_data` when a new internal playback starts.

## Included Implementations

Two implementations are included with the module, `AudioSpatializer3D` and `AudioSpatializerEffect`. 

### AudioSpatializer3D

`AudioSpatializer3D` replicates the behavior of the `AudioStreamPlayer3D`. It can be used as a reference when developing GDExtension based implementations. Most of the code was copied from `AudioStreamPlayer3D`, with a few fixes and changes, and made to work within the `AudioSpatializer` framework. The flag `mix_channel_mode` shows how the different methods to override in `AudioSpatializer` differ in how they should be implemented.

### AudioSpatializerEffect

`AudioSpatializerEffect` is a partial implementation that needs to be extended to be used. It includes the ability to add `AudioEffect`s to sounds coming from the `AudioSpatializer`, before it gets mixed into the target audio bus(es). It handles the lowest level audio processing in fast C++ code, while allowing for GDScript/C# extensions that do the spatialization calculation and audio bus selection.

To extend it, you need to extend `AudioSpatializerEffect`, `AudioSpatializerInstanceEffect`, and optionally `SpatializerParameters`. 

### AudioSpatializerEffect

You need to implement `_instantiate`, to create a copy of your custom `AudioSpatializerInstanceEffect`, and initialize it. Adding any extra `AudioEffect`s to the instance in the `_instantiate` will preserve them at the front of the `audio_effects` list. Any `AudioEffect`s added via the editor will be appended.

### AudioSpatializerInstanceEffect

Both `_process_frames` and `_mix_channel` are disable, and don't need to be implemented. Only `_calculate_spatialization` is required, in order to populate the `SpatializerParameters` properties.

An extra method is introduced to make configuration changes to the `audio_effects`: `_process_effects`. It happens at the start of `_process_frames`, just before all the `audio_effects` are applied to the audio stream. This is needed since modifying the `audio_effects` from the physics thread (i.e. `_calculate_spatialization`) can result in undefined behavior.

## Implementation Details

### Overview

The Godot Audio system, from a high level, has a few distinct stages (note: this describes Stream Playback, not the Sample Playback Type, which works differently):

1. Sound sources (in the form of `AudioStreamPlayer`, or the 2D/3D players) can play a sound, which will send an `AudioStreamPlayback` object to the `AudioServer`, with a specific start time, a map of audio bus names and associated volumes, and the playback pitch to use when sampling the audio stream. During normal gameplay, the audio player can update the bus names and volumes, and the pitch, as well as stop or pause the audio stream.
2. In the `AudioServer` main loop (in `_mix_step`), all active `AudioStreamPlayback`s have `mix` called on them to sample an array of audio frame data (using the current pitch for that playback). These frames are mixed into the specified audio bus channel buffers, using the specified volumes from that playback. There is also an optional highshelf filter applied per channel (similar to a lowpass filter).
3. Once all `AudioStreamPlayback`s have been mixed into the different audio bus channel buffers, `AudioServer` starts looping over the audio buses, from highest to lowest (i.e. right to left in the audio bus layout panel in the editor). For each audio bus, it loops over any `AudioEffect`s on the bus, applying them to the channel buffers.
4. After all `AudioEffect`s are processed, the channel buffers for that bus are mixed into their `send` audio bus (as defined in the audio bus layout). Then the next audio bus is mixed in step 3, until all buses are mixed, down to `Master`.
5. The `Master` audio bus channel buffers are sent to the audio driver.

### High Level Explanation

Basically none of the above stages are accessible from any of the built in extension mechanisms (GDScript, C#, or GDExtensions). `AudioServer` doesn't expose any of the playback methods, `AudioStreamPlayer` classes don't allow for changing their `_update_panning` methods, and the way `AudioServer` mixes the audio into the audio bus channel buffers isn't exposed either (`_mix_step_for_channel`). Since there seems to be no way in using the normal extensions, this module makes its own, partially bypassing some of what the `AudioServer` is doing.

To bypass the limitations of not having access to the `AudioServer` main loop, this module basically provides a mechanism to calculate your own spatialer parameters and volumes, and pre-mix the audio of your sounds, which are then passed to the `AudioServer` via a custom proxy `AudioStreamPlayback`.

When an audio player plays a sound, an internal `AudioStreamPlayback` is created, and one (or more) proxy `AudioStreamPlayback`s is created and sent to the `AudioServer`. When `AudioServer` tries to sample from our proxy playbacks (in step 2 above), the associated `AudioSpatializerInstance` will handle its own internal sampling, then mix the audio itself, and return the mixed audio to the `AudioServer`. At that point, it goes through the normal path, getting mixed into the target audio bus(es), having `AudioEffect`s applied, and finally being sent to the driver.

### Custom Mix Channels

There are two main modes that `AudioSpatializerInstance` can use: with custom mix channels, or not (set by overriding the return value of `_should_mix_channels`). 

When _not_ using custom mix channels, one proxy `AudioStreamPlayback` is created for the `AudioServer`. All active internal playbacks get sampled (using the pitch from `SpatializerParameters`) and mixed into a single mix buffer, and returned to the `AudioServer` when `AudioStreamPlayback.mix()` is called. No explicit channel dependent mixing is done, only fading out playbacks that are ending. Audio bus names and volumes are forwarded to the `AudioServer`, which then does the normal mixing using the mix buffer.

When custom mix channels is enabled, `AudioSpatializerInstance` creates up to four proxy `AudioStreamPlayback`s, one for each stereo audio channel. When mixing, the `_mix_channel` method is called to process each internal playback stream (one for each channel), which are added to the mix buffers. It is expected to do the volume adjustments itself, in most cases using the `mix_volumes` spatial parameter (and possibly a `prev_volume`, which can be stored in `SpatializerPlaybackData`). This can result in up to four internal mix buffers (depending on the number of channels of the audio driver), which are then passed to the `AudioServer` via the proxy `AudioStreamPlayback`s.

Since each proxy playback can only provide one channel of data, each proxy masks the audio bus volume vectors to only play their channel. Also, since the mix buffers are pre-mixed, it doesn't pass the volumes as is, but instead normalizes the audio bus volumes by the stored mix_volumes that the mix buffer was mixed with. This means that the 'main' audio bus will get volumes that are all 1.0 or 0.0, depending on the channel. But any secondary audio buses will get a relative volume that can be used by the `AudioServer` to further mix into the secondary buses.
