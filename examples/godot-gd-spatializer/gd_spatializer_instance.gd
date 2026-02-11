extends AudioSpatializerInstanceEffect
class_name GDSpatializerInstance

const CMP_EPSILON := 0.000001
var base: GDSpatializer

func calc_stereo_volume(source_dir: Vector3, pan_strength: float) -> Vector2:
	var flatrad := sqrt(source_dir.x * source_dir.x + source_dir.z * source_dir.z);
	if is_zero_approx(flatrad):
		flatrad = 1.0
	var g: float = clamp((1.0 - pan_strength) * (1.0 - pan_strength), 0.0, 1.0);
	var f: float = (1.0 - g) / (1.0 + g);
	var cosx: float = clamp(source_dir.x / flatrad, -1.0, 1.0)
	var fcosx: float = cosx * f;
	return Vector2(sqrt((-fcosx + 1.0) / 2.0), sqrt((fcosx + 1.0) / 2.0));

func get_attenuation_db(distance: float) -> float:
	# Inverse attenuation model
	var att: float = linear_to_db(1.0 / ((distance / base.unit_size) + CMP_EPSILON));
	att += get_audio_player().volume_db
	if att > get_audio_player().max_db:
		att = get_audio_player().max_db
	return att

func get_overriding_area() -> Area3D:
	var world_3d := get_audio_player().get_world_3d()
	var global_pos = get_audio_player().global_position
	var point_params := PhysicsPointQueryParameters3D.new()
	point_params.position = global_pos
	point_params.collision_mask = base.area_mask
	point_params.collide_with_bodies = false
	point_params.collide_with_areas = true
	
	var areas := world_3d.direct_space_state.intersect_point(point_params)
	for sr: Dictionary in areas:
		if not sr.collider:
			continue
		
		var tarea: Area3D = sr.collider as Area3D
		if not tarea:
			continue
		
		if not tarea.audio_bus_override and not tarea.reverb_bus_enabled:
			continue
		
		return tarea
	return null

func calc_reverb_vol(area: Area3D, listener_area_pos: Vector3, direct_vol: PackedVector2Array) -> PackedVector2Array:
	var reverb_vol := PackedVector2Array()
	reverb_vol.resize(4)
	reverb_vol.fill(Vector2.ZERO)
	
	var uniformity := area.reverb_bus_uniformity
	var area_send := area.reverb_bus_amount
	
	if uniformity > 0.0:
		var distance := listener_area_pos.length()
		var attenuation := db_to_linear(get_attenuation_db(distance))
		var center_val: PackedFloat32Array = [0.5,  0.25, 0.16666 ,0.125]
		var bus_idx := AudioServer.get_bus_index(area.reverb_bus_name)
		var chan_count := AudioServer.get_bus_channels(bus_idx)
		var center_frame := Vector2(center_val[chan_count - 1], center_val[chan_count - 1])
		
		if attenuation < 1.0:
			var rev_pos := listener_area_pos
			rev_pos.y = 0.0
			rev_pos = rev_pos.normalized()
			
			reverb_vol[0] = calc_stereo_volume(rev_pos, base.panning_strength)
			
			for i in chan_count:
				reverb_vol[i] = reverb_vol[i].lerp(center_frame, attenuation)
		else:
			for i in chan_count:
				reverb_vol[i] = center_frame
		
		for i in chan_count:
			reverb_vol[i] = direct_vol[i].lerp(reverb_vol[i] * attenuation, uniformity)
			reverb_vol[i] *= area_send
	else:
		for i in range(4):
			reverb_vol[i] = direct_vol[i] * area_send
	return reverb_vol

func _calculate_spatialization() -> SpatializerParameters:
	var params := GDSpatializerParameters.new()
	var mix_volumes := PackedVector2Array()
	mix_volumes.resize(4)
	var audio_player := get_audio_player()
	var global_pos := audio_player.global_position
	var listener := audio_player.get_viewport().get_camera_3d()
	
	var local_pos: Vector3 = listener.global_transform.orthonormalized().affine_inverse() * global_pos
	var dist := local_pos.length()
	var att_base := get_attenuation_db(dist)
	var multiplier: float = db_to_linear(att_base)
	if base.max_distance > 0:
		multiplier *= max(0, 1.0 - (dist / base.max_distance))
	
	var db_att: float = (1.0 - min(1.0, multiplier)) * base.attenuation_filter_db
	params.gain = db_to_linear(db_att)
	
	mix_volumes[0] = multiplier * calc_stereo_volume(local_pos, base.panning_strength)
	params.mix_volumes = mix_volumes
	
	var area: Area3D = get_overriding_area()
	if area:
		if area.audio_bus_override:
			params.add_bus_volume(area.audio_bus_name, mix_volumes)
		else:
			params.add_bus_volume(audio_player.bus, mix_volumes)
		
		if area.reverb_bus_enabled:
			var listener_area_pos := get_listener_area_pos(area, listener.global_transform)
			var reverb_vol := calc_reverb_vol(area, listener_area_pos, mix_volumes)
			params.add_bus_volume(area.reverb_bus_name, reverb_vol)
	else:
		params.add_bus_volume(audio_player.bus, mix_volumes)
	
	params.pitch_scale = audio_player.pitch_scale
	params.update_parameters = true
	return params

func _process_effects(spatial_parameters: SpatializerParameters, _playback_data: SpatializerPlaybackData) -> void:
	var filter := audio_effects[0] as AudioEffectHighShelfFilter
	filter.gain = (spatial_parameters as GDSpatializerParameters).gain

func get_listener_area_pos(area: Area3D, listener_xform: Transform3D) -> Vector3:
	var listener_area_pos: Vector3
	if area.reverb_bus_uniformity > 0:
		# Why isn't PhysicsDirectSpaceState3D.get_closest_point_to_object_volume exposed in GDScript?!
		# Use recursive intersect_ray's instead... has the potential to be kinda inefficient
		var ray_params := PhysicsRayQueryParameters3D.new()
		ray_params.collide_with_areas = true
		ray_params.collide_with_bodies = false
		#ray_params.hit_from_inside = true
		ray_params.from = listener_xform.origin
		ray_params.to = get_audio_player().global_position
		ray_params.exclude = []
		ray_params.collision_mask = area.collision_mask
		while true:
			var ret := area.get_world_3d().direct_space_state.intersect_ray(ray_params)
			if ret:
				if ret.collider == area:
					# Found our area!
					var area_sound_pos = ret.position
					listener_area_pos = listener_xform.orthonormalized().affine_inverse() * area_sound_pos
					break
				else:
					ray_params.from = ret.position
					ray_params.exclude.append(ret.collider.get_rid())
					continue
			else:
				# no intersection, reached audio_player position: must be inside the volume, 
				# stop searching, and leave listener_area_pos as zero.
				break
	return listener_area_pos
