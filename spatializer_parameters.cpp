/**************************************************************************/
/*  spatializer_results.cpp                                               */
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

#include "spatializer_parameters.h"
#include "audio_spatializer.h"
#include "servers/audio/audio_server.h"

void SpatializerParameters::add_bus_volume(const StringName p_bus, Vector<Vector2> p_volumes) {
	ERR_FAIL_COND(p_volumes.size() != AudioServer::MAX_CHANNELS_PER_BUS);
	bus_volumes[p_bus] = p_volumes;
}

Dictionary SpatializerParameters::get_bus_volumes() const {
	return bus_volumes;
}

void SpatializerParameters::set_mix_volumes(Vector<Vector2> p_volumes) {
	ERR_FAIL_COND(p_volumes.size() != AudioServer::MAX_CHANNELS_PER_BUS);
	mix_volumes = p_volumes;
}

Vector<Vector2> SpatializerParameters::get_mix_volumes() const {
	return mix_volumes;
}

void SpatializerParameters::set_pitch_scale(float p_pitch_scale) {
	pitch_scale = p_pitch_scale;
}

float SpatializerParameters::get_pitch_scale() const {
	return pitch_scale;
}

void SpatializerParameters::set_update_parameters(bool p_update) {
	update_parameters = p_update;
}

bool SpatializerParameters::should_update_parameters() const {
	return update_parameters;
}

void SpatializerParameters::_bind_methods() {
	ClassDB::bind_method(D_METHOD("add_bus_volume", "bus", "volumes"), &SpatializerParameters::add_bus_volume);
	ClassDB::bind_method(D_METHOD("get_bus_volumes"), &SpatializerParameters::get_bus_volumes);

	ClassDB::bind_method(D_METHOD("set_mix_volumes", "volumes"), &SpatializerParameters::set_mix_volumes);
	ClassDB::bind_method(D_METHOD("get_mix_volumes"), &SpatializerParameters::get_mix_volumes);

	ClassDB::bind_method(D_METHOD("set_pitch_scale", "pitch_scale"), &SpatializerParameters::set_pitch_scale);
	ClassDB::bind_method(D_METHOD("get_pitch_scale"), &SpatializerParameters::get_pitch_scale);

	ClassDB::bind_method(D_METHOD("set_update_parameters", "update"), &SpatializerParameters::set_update_parameters);
	ClassDB::bind_method(D_METHOD("should_update_parameters"), &SpatializerParameters::should_update_parameters);

	ADD_PROPERTY(PropertyInfo(Variant::PACKED_VECTOR2_ARRAY, "mix_volumes"), "set_mix_volumes", "get_mix_volumes");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "pitch_scale", PROPERTY_HINT_RANGE, "0.01,4.0,0.01"), "set_pitch_scale", "get_pitch_scale");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "update_parameters"), "set_update_parameters", "should_update_parameters");
}

///////////////////////////////////////////////////////////////////////////////
