# Godot 4.7.2 with Regolith's renderer patches, double
# precision: the editor and Windows export templates.
# Usage: engine\build_godot.ps1 [DEST]  (see engine/README.md)
param([string]$Dest = "")

$ErrorActionPreference = "Stop"
$GodotTag = "4.7.2-stable"
$GodotRepo = "https://github.com/godotengine/godot.git"
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
if ($Dest -eq "") { $Dest = Join-Path $Here "godot" }
$Jobs = if ($env:JOBS) { $env:JOBS } else { $env:NUMBER_OF_PROCESSORS }

function Die($msg) { Write-Error "build_godot: $msg"; exit 1 }

foreach ($tool in @("git", "python", "scons")) {
	if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) { Die "$tool is required" }
}

if (-not (Test-Path (Join-Path $Dest ".git"))) {
	Write-Host "== cloning Godot $GodotTag into $Dest"
	git clone --depth 1 --branch $GodotTag $GodotRepo $Dest
	if ($LASTEXITCODE -ne 0) { Die "clone failed" }
}
Set-Location $Dest
# the patches are made for this one tag
$tag = git describe --tags --exact-match HEAD
if ($tag -ne $GodotTag) { Die "$Dest is at $tag, expected $GodotTag" }

Write-Host "== patching"
foreach ($p in Get-ChildItem (Join-Path $Here "patches\*.patch") | Sort-Object Name) {
	git apply --reverse --check $p.FullName *> $null
	if ($LASTEXITCODE -eq 0) { Write-Host "   already applied: $($p.Name)"; continue }
	git apply --check $p.FullName
	if ($LASTEXITCODE -ne 0) { Die "$($p.Name) does not apply; start from a clean $GodotTag checkout" }
	git apply $p.FullName
	Write-Host "   applied: $($p.Name)"
}

$env:BUILD_NAME = "regolith"
foreach ($target in @("editor", "template_debug", "template_release")) {
	Write-Host "== building $target"
	scons platform=windows arch=x86_64 precision=double target=$target -j $Jobs
	if ($LASTEXITCODE -ne 0) { Die "$target build failed" }
}

# where the editor looks: <templates>\4.7.2.stable.double\
$Version = $GodotTag.Replace("-stable", "") + ".stable.double"
$TDir = Join-Path $env:APPDATA "Godot\export_templates\$Version"
New-Item -ItemType Directory -Force -Path $TDir | Out-Null
Copy-Item "bin\godot.windows.template_debug.double.x86_64.exe" (Join-Path $TDir "windows_debug_x86_64.exe")
Copy-Item "bin\godot.windows.template_release.double.x86_64.exe" (Join-Path $TDir "windows_release_x86_64.exe")
Set-Content -Path (Join-Path $TDir "version.txt") -Value $Version

Write-Host ""
Write-Host "Done."
Write-Host "  editor:    $Dest\bin\godot.windows.editor.double.x86_64.exe"
Write-Host "  templates: $TDir"
Write-Host "Open your project with that editor; Regolith runs at full speed."
