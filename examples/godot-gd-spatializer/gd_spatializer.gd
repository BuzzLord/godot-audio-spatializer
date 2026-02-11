extends AudioSpatializerEffect
class_name GDSpatializer

@export_range(0, 1000, 0.1) var max_distance: float = 0.0
@export_range(1, 20500, 1) var attenuation_cutoff_hz: float = 5000.0
@export_range(-80, 0, 0.1) var attenuation_filter_db: float = -24.0
@export_range(0.1, 100, 0.1) var unit_size: float = 10.0
@export_range(0, 3, 0.01) var panning_strength: float = 1.0
@export_flags_3d_physics var area_mask: int = 0xFFFFFFFF

func _instantiate_effect() -> AudioSpatializerInstanceEffect:
	var ins := GDSpatializerInstance.new()
	ins.base = self
	var filter := AudioEffectHighShelfFilter.new()
	filter.cutoff_hz = attenuation_cutoff_hz
	filter.db = AudioEffectFilter.FILTER_6DB
	filter.resonance = 1.0
	filter.gain = 0.0
	ins.audio_effects = [filter]
	return ins
