# T1 of the organizer sim profile on a Windows laptop: Isaac Sim 5.1.0 (Windows build) + Pegasus
# v5.1.0 running the organizer's sim/pegasus_skybeta.py through isaac/pegasus_skybeta_course.py.
# PX4 SITL and the stack run in WSL2 (run_px4.sh / run_stack.sh) with networkingMode=mirrored,
# so localhost TCP 4560 (Pegasus <-> PX4) and DDS (UDP only, fastdds_udp_only.xml) cross over.
#   powershell -File run_isaac.ps1 [-Course] [-Textured] [-Log <file>]
param([switch]$Course, [switch]$Textured, [string]$Log = "")
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Repo = Split-Path -Parent (Split-Path -Parent $Here)   # repo root (isaac_sim/windows -> ..\..)
if (-not $env:ISAACSIM_PATH) { $env:ISAACSIM_PATH = 'C:\isaacsim' }
if (-not $env:SKYBETA_ORGANIZER_SCRIPT) {
  $env:SKYBETA_ORGANIZER_SCRIPT = Join-Path $Repo 'skybeta\sim\pegasus_skybeta.py' }
# Isaac's bundled ROS 2 Jazzy (matches WSL); Pegasus imports rclpy from it.
$env:ROS_DISTRO = 'jazzy'
$env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
$env:PATH = "$env:ISAACSIM_PATH\exts\isaacsim.ros2.bridge\jazzy\lib;$env:PATH"
if (-not $env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID = '55' }
$env:FASTRTPS_DEFAULT_PROFILES_FILE = Join-Path (Split-Path -Parent $Here) 'fastdds_udp_only.xml'
$env:FASTDDS_DEFAULT_PROFILES_FILE = $env:FASTRTPS_DEFAULT_PROFILES_FILE
# laptop: D3D12 renderer (Vulkan RTX crashes in rtx.scenedb on RTX 5070 Ti Laptop / driver 621)
if (-not $env:ISAAC_KIT_ARGS) { $env:ISAAC_KIT_ARGS = '--/app/vulkan=false' }
if (-not $env:SKYBETA_HEADLESS) { $env:SKYBETA_HEADLESS = '1' }
if (-not $env:SKYBETA_CAMERA_X) { $env:SKYBETA_CAMERA_X = '0.10' }
if (-not $env:SKYBETA_CAMERA_VIEW_ROLL_DEG) { $env:SKYBETA_CAMERA_VIEW_ROLL_DEG = '180' }
if ($Course) {
  $env:SKYBETA_ENV = 'Default Environment'
  $env:SKYBETA_SPAWN_X = '-2.515'; $env:SKYBETA_SPAWN_Y = '0.914'; $env:SKYBETA_SPAWN_YAW = '1.5707963267948966'
  if (-not $env:SKYBETA_COURSE_USD) {
    $env:SKYBETA_COURSE_USD = if ($Textured) { 'C:/isaacsim-dl/course/course_textured.usd' } else { 'C:/isaacsim-dl/course/course.usd' } }
}
$script = Join-Path (Split-Path -Parent $Here) 'isaac\pegasus_skybeta_course.py'
if ($Log) { & "$env:ISAACSIM_PATH\python.bat" $script *> $Log } else { & "$env:ISAACSIM_PATH\python.bat" $script }
