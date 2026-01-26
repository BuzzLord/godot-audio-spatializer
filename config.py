def can_build(env, platform):
    return True


def configure(env):
    pass


# def get_icons_path():
#     return "icons"


def get_doc_classes():
    return [
        "AudioSpatializer",
        "AudioSpatializerInstance",
        "SpatializerParameters",
        "SpatializerPlaybackData",
        "AudioStreamPlayerSpatial",
        "AudioSpatializer3D",
        "AudioSpatializerInstance3D",
        "SpatializerParameters3D",
        "SpatializerPlaybackData3D",
        "AudioSpatializerEffect",
        "AudioSpatializerInstanceEffect",
        "SpatializerPlaybackDataEffect",
    ]


def get_doc_path():
    return "doc_classes"
