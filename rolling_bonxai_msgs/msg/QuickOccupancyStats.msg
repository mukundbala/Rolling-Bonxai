# Header
std_msgs/Header header

# Memory usage (MiB = 1024^2 bytes)
float64 total_memory_mib
float64 grid_memory_mib
float64 buffer_memory_mib

# Cell counts
uint64 total_active_cells

# Update statistics
uint64 total_insertions
uint64 hit_points_added
uint64 miss_points_added

# Performance metrics
uint64 point_clouds_processed  # scans inserted into the map since startup

# Flags
bool has_data
