# This Source Code Form is subject to the terms of the
# Mozilla Public License, v. 2.0. If a copy of the MPL was
# not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

extends CharacterBody3D
## Regolith's example flyer: the system's focus, flown with the
## mouse and keys from the ground to orbit and between planets.
## Speed follows the height over the ground, so one key covers
## a walk and an interplanetary hop alike.

@export var system_path: NodePath
@export var hud_path: NodePath

var system: RegolithSystem
var hud: Label
var cam: Camera3D
var yaw := 0.0
var pitch := -0.2
var placed := false

# key, label, body, latitude, longitude (or the sun's elevation
# there when "sun" is set: the spot is found in daylight), height
# over the ground (m), and what to look at
const JUMPS := [
	[KEY_1, "Mars ground", "Mars", -5.0, 25.0, 30.0, "sun", "horizon"],
	[KEY_2, "Mars orbit", "Mars", 0.0, 35.0, 400000.0, "sun", "limb"],
	[KEY_3, "Mercury ground", "Mercury", 10.0, 15.0, 30.0, "sun", "horizon"],
	[KEY_4, "Mercury orbit", "Mercury", 0.0, 35.0, 300000.0, "sun", "limb"],
	[KEY_5, "Mars from Phobos", "Phobos", 0.0, 0.0, 30.0, "", "Mars"],
]

func _ready() -> void:
	system = get_node(system_path)
	hud = get_node_or_null(hud_path)
	cam = $Camera3D
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

func _unhandled_input(e: InputEvent) -> void:
	if e is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		yaw -= e.relative.x * 0.003
		pitch = clampf(pitch - e.relative.y * 0.003, -1.55, 1.55)
	elif e is InputEventKey and e.pressed:
		if e.keycode == KEY_ESCAPE:
			Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		for j in JUMPS:
			if e.keycode == j[0]:
				_jump(j)
	elif e is InputEventMouseButton and e.pressed:
		Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

func _jump(j: Array) -> void:
	var lat: float = j[3]
	var lon: float = j[4]
	if j[6] == "sun":
		# there first: the sun stands elsewhere from another planet
		system.put_near(self, j[2], lat, 0.0, j[5])
		lon = _lon_for_sun(j[2], lat, j[4])
	system.put_near(self, j[2], lat, lon, j[5])
	var up: Vector3 = system.get_up(global_position)
	var look := -global_basis.z
	match j[7]:
		"horizon":
			pitch = -0.08
			return
		"limb":
			pitch = -0.5
			return
		_:
			# a body's centre, wherever it stands in the sky
			look = (system.get_body_point(j[7], 0.0, 0.0, 0.0) - global_position).normalized()
	var flat := look - up * look.dot(up)
	if flat.length() > 1e-3:
		global_basis = Basis.looking_at(flat.normalized(), up)
	pitch = asin(clampf(look.dot(up), -1.0, 1.0))
	yaw = 0.0

## The longitude on this latitude where the sun stands nearest
## `elevation` degrees, in the morning (sun to the east).
func _lon_for_sun(body: String, lat: float, elevation: float) -> float:
	var sun: Vector3 = system.get_sun_direction()
	var centre: Vector3 = system.get_body_point(body, 0.0, 0.0, 0.0)
	var best := 0.0
	var best_err := 1e9
	for k in 72:
		var lon := -180.0 + k * 5.0
		var up: Vector3 = (system.get_body_point(body, lat, lon, 1.0e6) - centre).normalized()
		var east: Vector3 = (system.get_body_point(body, lat, lon + 1.0, 1.0e6) - centre).normalized() - up
		var el := rad_to_deg(asin(clampf(sun.dot(up), -1.0, 1.0)))
		var err := absf(el - elevation) + (0.0 if sun.dot(east) > 0.0 else 1000.0)
		if err < best_err:
			best_err = err
			best = lon
	return best

func _physics_process(dt: float) -> void:
	if system.is_anchored() == false:
		return
	if placed == false:
		placed = true
		_jump(JUMPS[1])
	var up: Vector3 = system.get_up(global_position)
	# heading on the local horizon, then the pitch
	var fwd0 := global_basis.z * -1.0
	var flat := fwd0 - up * fwd0.dot(up)
	if flat.length() < 1e-3:
		flat = global_basis.y - up * global_basis.y.dot(up)
	global_basis = Basis.looking_at(flat.normalized(), up).rotated(up, yaw)
	yaw = 0.0
	cam.rotation = Vector3(pitch, 0.0, 0.0)

	var clearance: float = system.get_ground_clearance(global_position)
	var h := clearance if is_finite(clearance) else 1.0e6
	# up to 5e10 m/s out between the planets: an AU in seconds
	var speed := clampf(h * 0.6, 4.0, 5.0e10)
	if Input.is_key_pressed(KEY_SHIFT):
		speed *= 5.0
	var cb := cam.global_basis
	var dir := Vector3.ZERO
	if Input.is_key_pressed(KEY_W):
		dir -= cb.z
	if Input.is_key_pressed(KEY_S):
		dir += cb.z
	if Input.is_key_pressed(KEY_A):
		dir -= cb.x
	if Input.is_key_pressed(KEY_D):
		dir += cb.x
	if Input.is_key_pressed(KEY_SPACE):
		dir += up
	if Input.is_key_pressed(KEY_CTRL):
		dir -= up
	if dir != Vector3.ZERO:
		var step := dir.normalized() * speed * dt
		# never into the ground
		if is_finite(clearance) and step.dot(up) < 0.0 and clearance + step.dot(up) < 2.0:
			step -= up * step.dot(up)
		global_position += step
	# at rest in the frame, so the floating origin settles
	velocity = system.get_frame_velocity() * -1.0
	# "sunny 16": the camera's base exposure set for this
	# sunlight (26x at Mercury), auto exposure fine-tunes it
	var attr := get_viewport().get_camera_3d().attributes
	if attr == null:
		attr = get_viewport().world_3d.camera_attributes
	if attr:
		attr.exposure_multiplier = 1.0 / maxf(system.get_sun_scale(), 1e-3)
	_hud(h, speed)

func _hud(h: float, speed: float) -> void:
	if hud == null:
		return
	var alt := "%.0f m" % h if h < 10000.0 else "%.1f km" % (h / 1000.0)
	hud.text = "%s  |  %s over the ground  |  %.0f m/s  |  local time %.1f h\n" % [
		system.get_dominant_name(), alt, speed, system.get_local_solar_time(global_position)]
	hud.text += "mouse look, WASD fly, Space up, Ctrl down, Shift fast\n"
	for j in JUMPS:
		hud.text += "%s %s   " % [OS.get_keycode_string(j[0]), j[1]]
	hud.text += "Esc mouse"
