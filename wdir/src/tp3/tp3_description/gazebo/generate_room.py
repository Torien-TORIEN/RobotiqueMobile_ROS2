import random


def get_header():
    return """
<?xml version='1.0'?>
<sdf version="1.9">
    <model name='room'>
        <static>true</static>
"""

def get_footer():
    return """
    </model>
</sdf>
"""

def get_wall(name:str, pose:str, wall_length:float, wall_height:float):
    color = "0.0 1.0 1.0 1"
    wall_width = 0.5
    return f"""
            <link name='{name}'>
                <pose relative_to='__model__'>{pose}</pose>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>{wall_width} {wall_length} {wall_height}</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>{color}</ambient>
                        <diffuse>{color}</diffuse>
                        <specular>{color}</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>{wall_width} {wall_length} {wall_height}</size>
                        </box>
                    </geometry>
                </collision>
            </link>
"""


def generate_room(sdffile_name:str):
    room_width = 10
    wall_height = 2
    door_size = 2

    poses:list
    names:list
    lengths:list
    with open(sdffile_name, "w") as room:
        room.write(get_header())

        door = random.randint(0,3)
        wall1_size = random.randint(1, room_width-door_size-1)
        wall2_size = room_width - wall1_size - door_size
        for side in range(4):
            match side:
                case 0:
                    if door != side:
                        poses = [f"{room_width/2} 0 {wall_height/2} 0 0 0"]
                        names = ["wNorth"]
                        lengths = [room_width]
                    else:
                        poses = [f"{room_width/2} -{room_width/2-wall1_size/2} {wall_height/2} 0 0 0", f"{room_width/2} {room_width/2-wall2_size/2} {wall_height/2} 0 0 0"]
                        names = ["wNorth1", "wNorth2"]
                        lengths = [wall1_size, wall2_size]
                case 1:
                    if door != side:
                        poses = [f"0 {room_width/2} {wall_height/2} 0 0 1.570796327"]
                        names = ["wWest"]
                        lengths = [room_width]
                    else:
                        poses = [f"-{room_width/2-wall1_size/2} {room_width/2} {wall_height/2} 0 0 1.570796327", f"{room_width/2-wall2_size/2} {room_width/2} {wall_height/2} 0 0 1.570796327"]
                        names = ["wWest1", "wWest2"]
                        lengths = [wall1_size, wall2_size]
                case 2:
                    if door != side:
                        poses = [f"-{room_width/2} 0 {wall_height/2} 0 0 0"]
                        names = ["wSouth"]
                        lengths = [room_width]
                    else:
                        poses = [f"-{room_width/2} -{room_width/2-wall1_size/2} {wall_height/2} 0 0 0", f"-{room_width/2} {room_width/2-wall2_size/2} {wall_height/2} 0 0 0"]
                        names = ["wSouth1", "wSouth2"]
                        lengths = [wall1_size, wall2_size]
                case 3:
                    if door != side:
                        poses = [f"0 -{room_width/2} {wall_height/2} 0 0 1.570796327"]
                        names = ["wEast"]
                        lengths = [room_width]
                    else:
                        poses = [f"-{room_width/2-wall1_size/2} -{room_width/2} {wall_height/2} 0 0 1.570796327", f"{room_width/2-wall2_size/2} -{room_width/2} {wall_height/2} 0 0 1.570796327"]
                        names = ["wEast1", "wEast2"]
                        lengths = [wall1_size, wall2_size]

            for (name, pose, length) in zip(names, poses, lengths):
                room.write(get_wall(pose=pose, name=name, wall_length=length, wall_height=wall_height))

        room.write(get_footer())


if __name__ == "__main__":
    sdffile_name = "tp3_room.sdf"
    generate_room(sdffile_name)
