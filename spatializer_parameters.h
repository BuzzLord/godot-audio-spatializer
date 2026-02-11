/**************************************************************************/
/*  spatializer_results.h                                                 */
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

#include "core/io/resource.h"
#include "core/templates/vector.h"
#include "core/variant/dictionary.h"

struct AudioFrame;

class SpatializerParameters : public RefCounted {
	GDCLASS(SpatializerParameters, RefCounted);

private:
	// Dictionary of bus -> volume vectors to output. Volumes will be divided by mix_volumes before sending to AudioServer, to account for mixing done in AudioSpatializer.
	Dictionary bus_volumes;
	// Vector of volumes (size 4) that should be used by AudioSpatializer to mix the audio frames to, for each channel.
	Vector<Vector2> mix_volumes;
	// Pitch scale used when mixing frames from the stream playback.
	float pitch_scale = 1.0f;
	// Should be set to true if volumes were updated; will cause AudioServer to update the volumes. Useful when sound is outside max_distance, and you don't want to keep updating an empty set of bus volumes.
	bool update_parameters = false;

protected:
	static void _bind_methods();

public:
	void add_bus_volume(const StringName p_bus, Vector<Vector2> p_volumes);
	Dictionary get_bus_volumes() const;

	void set_mix_volumes(Vector<Vector2> p_volumes);
	Vector<Vector2> get_mix_volumes() const;

	void set_pitch_scale(float p_pitch_scale);
	float get_pitch_scale() const;

	void set_update_parameters(bool p_update);
	bool should_update_parameters() const;
};

class SpatializerPlaybackData : public RefCounted {
	GDCLASS(SpatializerPlaybackData, RefCounted);
};
