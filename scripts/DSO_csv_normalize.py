python3 -c '
with open("/root/project/Datasets/TUM/rgbd_dataset_freiburg1_desk/times.txt") as f_times:
    timestamps = [line.strip().split()[0] for line in f_times if line.strip() and not line.startswith("#")]

with open("/root/project/results/traj_dso_fr1_desk.txt") as f_in, open("/root/project/results/traj_dso_fr1_desk_formatted.txt", "w") as f_out:
    for idx, line in enumerate(f_in):
        parts = line.strip().split()
        if len(parts) >= 8 and idx < len(timestamps):
            # Замінюємо 1-шу колонку (перше значення) на реальний timestamp камери
            parts[0] = timestamps[idx]
            f_out.write(" ".join(parts[:8]) + "\n")
'