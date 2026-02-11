extends Camera3D

func _physics_process(delta: float) -> void:
	var turn := Input.get_axis("ui_left", "ui_right")
	var move := Input.get_axis("ui_down", "ui_up")
	
	rotate_y(-turn * delta * 4.0)
	global_position += -global_basis.z * move * delta * 5.0
