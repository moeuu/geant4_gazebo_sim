## MeasurementRequest.msg
# 測定器の位置，遮蔽体の回転角，線源パラメータを持つ測定要求メッセージ
builtin_interfaces/Time stamp
geometry_msgs/Point detector_position          # 検出器の三次元位置
float32 shield_rotation_angle                  # 遮蔽体の回転角（ラジアン）
float32[] source_parameters                    # 線源パラメータ（[x,y,z,強度]など）
