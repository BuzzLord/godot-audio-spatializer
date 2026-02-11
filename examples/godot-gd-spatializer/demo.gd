extends Node3D

@onready var audio_stream_player_3d: AudioStreamPlayer3D = $AudioStreamPlayer3D
@onready var gd_audio_player_spatial: AudioStreamPlayerSpatial = $GDAudioPlayerSpatial
@onready var audio_player_spatial_3d: AudioStreamPlayerSpatial = $AudioPlayerSpatial3D

func _process(_delta: float) -> void:
	if Input.is_action_just_pressed("play_1"):
		audio_stream_player_3d.play()
	if Input.is_action_just_pressed("play_2"):
		gd_audio_player_spatial.play()
	if Input.is_action_just_pressed("play_3"):
		audio_player_spatial_3d.play()
	if Input.is_action_just_pressed("ui_accept"):
		if audio_stream_player_3d.playing:
			audio_stream_player_3d.stop()
		if gd_audio_player_spatial.playing:
			gd_audio_player_spatial.stop()
		if audio_player_spatial_3d.playing:
			audio_player_spatial_3d.stop()
	if Input.is_action_just_pressed("ui_cancel"):
		get_tree().quit(0)
