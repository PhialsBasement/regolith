extends SceneTree
## Builds Regolith's night sky data from public sources:
##   the Yale Bright Star Catalogue (BSC5, bsc5.dat), and NASA SVS
##   Deep Star Maps 2020's milkyway_2020_4k.exr (svs.gsfc.nasa.gov/4851).
## Writes regolith_stars.res and regolith_milkyway.res: Images in
## units of the Sun's illuminance (stars) and per steradian (sky).
## godot --headless --script make_sky_data.gd -- BSC5_DAT MILKYWAY_EXR OUT_DIR

const SUN_V := -26.74
# brightest Milky Way (the Sagittarius star cloud), V mag/arcsec2
const MW_PEAK_MAG := 20.0
const ARCSEC2_SR := 2.3504430539e-11
const STARS_W := 256
const MW_UNIT := 1e-9

func _init() -> void:
	var a := OS.get_cmdline_user_args()
	_stars(a[0], a[2].path_join("regolith_stars.res"))
	_milky_way(a[1], a[2].path_join("regolith_milkyway.res"))
	quit()

## Linear RGB of a blackbody, relative to the Sun's: G2 is white.
func _planck_rgb(t: float) -> Vector3:
	var c := Vector3()
	var sun := Vector3()
	var lams := [610e-9, 550e-9, 465e-9]
	for k in 3:
		var lam: float = lams[k]
		c[k] = 1.0 / (pow(lam, 5.0) * (exp(1.4388e-2 / (lam * t)) - 1.0))
		sun[k] = 1.0 / (pow(lam, 5.0) * (exp(1.4388e-2 / (lam * 5772.0)) - 1.0))
	c = c / sun
	return c / (0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z)

func _stars(src: String, dst: String) -> void:
	var f := FileAccess.open(src, FileAccess.READ)
	var rows: Array = []
	while f.get_position() < f.get_length():
		var line := f.get_line()
		if line.length() < 114 or line.substr(75, 2).strip_edges() == "" or line.substr(102, 5).strip_edges() == "":
			continue
		var ra := (float(line.substr(75, 2)) + float(line.substr(77, 2)) / 60.0 + float(line.substr(79, 4)) / 3600.0) * 15.0
		var dec := float(line.substr(84, 2)) + float(line.substr(86, 2)) / 60.0 + float(line.substr(88, 2)) / 3600.0
		if line.substr(83, 1) == "-":
			dec = -dec
		var v := float(line.substr(102, 5))
		var bv_text := line.substr(109, 5).strip_edges()
		var bv := float(bv_text) if bv_text != "" else 0.6
		rows.append([deg_to_rad(ra), deg_to_rad(dec), v, bv])
	var n := rows.size()
	var h := int(ceil(2.0 * n / STARS_W))
	var img := Image.create_empty(STARS_W, h, false, Image.FORMAT_RGBAF)
	for i in n:
		var r: Array = rows[i]
		var cd := cos(float(r[1]))
		var dir := Vector3(cd * cos(float(r[0])), cd * sin(float(r[0])), sin(float(r[1])))
		var flux := pow(10.0, -0.4 * (float(r[2]) - SUN_V))
		var bv := clampf(float(r[3]), -0.4, 2.0)
		var t := 4600.0 * (1.0 / (0.92 * bv + 1.7) + 1.0 / (0.92 * bv + 0.62))
		var col := _planck_rgb(t)
		img.set_pixel((2 * i) % STARS_W, (2 * i) / STARS_W, Color(dir.x, dir.y, dir.z, flux))
		img.set_pixel((2 * i + 1) % STARS_W, (2 * i + 1) / STARS_W, Color(col.x, col.y, col.z, float(r[2])))
	img.set_meta("count", n)
	ResourceSaver.save(img, dst)
	print("stars: %d from %s, brightest V %.2f" % [n, src.get_file(), rows.map(func(x): return x[2]).min()])

func _milky_way(src: String, dst: String) -> void:
	var img := Image.load_from_file(src)
	img.convert(Image.FORMAT_RGBF)
	img.resize(img.get_width() / 2, img.get_height() / 2, Image.INTERPOLATE_TRILINEAR)
	# the brightest 0.05% of the sky sets the calibration
	var lum: Array = []
	for y in range(0, img.get_height(), 3):
		for x in range(0, img.get_width(), 3):
			var c := img.get_pixel(x, y)
			lum.append(0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b)
	lum.sort()
	var peak: float = lum[int(lum.size() * 0.9995)]
	var target := pow(10.0, -0.4 * (MW_PEAK_MAG - SUN_V)) / ARCSEC2_SR
	# stored in units of MW_UNIT suns/sr: RGBE9995 bottoms out
	# near 1e-7, under the sky's real values
	var k := target / peak / MW_UNIT
	for y in img.get_height():
		for x in img.get_width():
			img.set_pixel(x, y, img.get_pixel(x, y) * k)
	img.convert(Image.FORMAT_RGBE9995)
	img.set_meta("unit", MW_UNIT)
	ResourceSaver.save(img, dst)
	var check: Image = load(dst)
	check.convert(Image.FORMAT_RGBF)
	var back := 0.0
	for x in range(0, check.get_width(), 4):
		back = maxf(back, check.get_pixel(x, check.get_height() / 2).get_luminance())
	print("milky way: %dx%d, peak %s -> %s suns/sr; stored peak %s units, read back %s on the middle row" % [
		img.get_width(), img.get_height(), str(peak), str(target), str(target / MW_UNIT), str(back)])
