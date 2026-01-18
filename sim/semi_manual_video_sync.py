from telemetry_video_sync import TelemetryVideoSync

# --------------------------------------------------
# CONFIG
# --------------------------------------------------

VIDEO_PATH = "data/video.mp4"
ULOG_PATH = "data/log.ulg"
CSV_PATH = "data/csv"

TELEMETRY_START_IDX = 16970
TELEMETRY_END_IDX = 63418

VIDEO_START_MESS = 22.31860479142235
VIDEO_END_MESS = 457.5700117272575

SAVE_EVERY_N = 1
PLOT_EVERY_N = 10


# --------------------------------------------------
# CREATE OBJECT
# --------------------------------------------------

sync = TelemetryVideoSync(
    telemetry_start_idx=TELEMETRY_START_IDX,
    telemetry_end_idx=TELEMETRY_END_IDX,
    video_start_time=VIDEO_START_MESS,
    video_end_time=VIDEO_END_MESS,
    video_path=VIDEO_PATH,
    ulog_path=ULOG_PATH,
    csv_path=CSV_PATH,
    save_every_n=SAVE_EVERY_N,
    plot_every_n=PLOT_EVERY_N,
)


# --------------------------------------------------
# PIPELINE
# --------------------------------------------------

# 1. Convert ULOG → CSV (run once)
sync.read_telemetry()

# 2. Load telemetry, cut, resample, load video frames
sync.analyze_telemetry()

# 3. Play synchronized telemetry video
sync.play_telemetry_video()

