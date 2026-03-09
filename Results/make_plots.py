#!/usr/bin/env python3
import os
import glob
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


RESULTS_DIR = "Results"
CSV_PATTERN = os.path.join(RESULTS_DIR, "trial_*.csv")
OUT_DIR = os.path.join(RESULTS_DIR, "plots")


def get_all_csvs():
    files = glob.glob(CSV_PATTERN)
    if not files:
        raise FileNotFoundError(f"No se encontraron CSVs con patrón: {CSV_PATTERN}")
    return sorted(files)

def get_ee_error(df):
    if "e_norm" in df.columns:
        return df["e_norm"].values
    return np.sqrt(
        (df["px"] - df["px_des"]) ** 2 +
        (df["py"] - df["py_des"]) ** 2 +
        (df["pz"] - df["pz_des"]) ** 2
    )


def plot_joint_tracking(df, out_dir, tag):
    t = df["t"].values

    fig, axs = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    axs = axs.flatten()

    for i in range(6):
        q_col = f"q{i+1}"
        qdes_col = f"qdes{i+1}"

        axs[i].plot(t, df[q_col], label=f"{q_col} real")
        axs[i].plot(t, df[qdes_col], "--", label=f"{qdes_col} des")
        axs[i].set_title(f"Joint {i+1} Tracking")
        axs[i].set_xlabel("Time (s)")
        axs[i].set_ylabel("Angle (rad)")
        axs[i].grid(True)
        axs[i].legend()

    fig.suptitle(f"Joint Tracking Plots ({tag})", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"joint_tracking_{tag}.png"), dpi=300)
    plt.close(fig)


def plot_task_space(df, out_dir, tag):
    t = df["t"].values

    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    components = [
        ("px", "px_des", "X"),
        ("py", "py_des", "Y"),
        ("pz", "pz_des", "Z"),
    ]

    for ax, (real_col, des_col, label) in zip(axs, components):
        ax.plot(t, df[real_col], label=f"{label} real")
        ax.plot(t, df[des_col], "--", label=f"{label} desired")
        ax.set_ylabel(f"{label} (m)")
        ax.set_title(f"Task-Space {label} Component")
        ax.grid(True)
        ax.legend()

    axs[-1].set_xlabel("Time (s)")
    fig.suptitle(f"Task-Space Component Plots ({tag})", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"task_space_xyz_{tag}.png"), dpi=300)
    plt.close(fig)


def plot_3d_path(df, out_dir, tag):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(df["px"], df["py"], df["pz"], label="Real path")
    ax.plot(df["px_des"], df["py_des"], df["pz_des"], "--", label="Desired path")

    ax.set_title(f"3D End-Effector Path ({tag})")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"path_3d_{tag}.png"), dpi=300)
    plt.close(fig)


def plot_ee_error(df, out_dir, tag):
    t = df["t"].values
    e_norm = get_ee_error(df)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, e_norm, label="EE position error")
    ax.set_title(f"End-Effector Error Over Time ({tag})")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"ee_error_{tag}.png"), dpi=300)
    plt.close(fig)


def plot_phase_portraits(df, out_dir, tag):
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    axs = axs.flatten()

    for i in range(4):
        q_col = f"q{i+1}"
        qd_col = f"qd{i+1}"

        axs[i].plot(df[q_col], df[qd_col])
        axs[i].set_title(f"Joint {i+1} Phase Portrait")
        axs[i].set_xlabel(f"{q_col} (rad)")
        axs[i].set_ylabel(f"{qd_col} (rad/s)")
        axs[i].grid(True)

    fig.suptitle(f"Phase Portraits ({tag})", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"phase_portraits_{tag}.png"), dpi=300)
    plt.close(fig)


def plot_joint_velocities(df, out_dir, tag):
    t = df["t"].values

    fig, axs = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    axs = axs.flatten()

    for i in range(6):
        qd_col = f"qd{i+1}"
        qd_des_col = f"qd_des{i+1}"

        axs[i].plot(t, df[qd_col], label=f"{qd_col} real")
        if qd_des_col in df.columns:
            axs[i].plot(t, df[qd_des_col], "--", label=f"{qd_des_col} des")
        axs[i].set_title(f"Joint {i+1} Velocity Tracking")
        axs[i].set_xlabel("Time (s)")
        axs[i].set_ylabel("Velocity (rad/s)")
        axs[i].grid(True)
        axs[i].legend()

    fig.suptitle(f"Joint Velocity Tracking ({tag})", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"joint_velocity_tracking_{tag}.png"), dpi=300)
    plt.close(fig)


def compute_joint_rmse(df):
    rmse = {}
    for i in range(6):
        q_col = f"q{i+1}"
        qdes_col = f"qdes{i+1}"
        err = df[q_col].values - df[qdes_col].values
        rmse[f"joint_{i+1}"] = float(np.sqrt(np.mean(err ** 2)))
    return rmse


def compute_joint_max_abs_error(df):
    max_err = {}
    for i in range(6):
        q_col = f"q{i+1}"
        qdes_col = f"qdes{i+1}"
        err = np.abs(df[q_col].values - df[qdes_col].values)
        max_err[f"joint_{i+1}"] = float(np.max(err))
    return max_err


def compute_ee_metrics(df):
    e_norm = get_ee_error(df)
    return {
        "rmse_ee_m": float(np.sqrt(np.mean(e_norm ** 2))),
        "max_ee_m": float(np.max(e_norm)),
        "mean_ee_m": float(np.mean(e_norm)),
    }


def get_dwell_mask(df):
    if "phase" not in df.columns:
        return None

    phase_str = df["phase"].astype(str).str.lower()
    mask = phase_str.str.contains("dwell")
    return mask


def compute_dwell_joint_mean_error(df):
    mask = get_dwell_mask(df)
    if mask is None or mask.sum() == 0:
        return None

    results = []
    dwell_df = df[mask].copy()

    for wp in sorted(dwell_df["wp_idx"].dropna().unique()):
        wp_df = dwell_df[dwell_df["wp_idx"] == wp]
        row = {"wp_idx": int(wp)}
        for i in range(6):
            q_col = f"q{i+1}"
            qdes_col = f"qdes{i+1}"
            err = np.abs(wp_df[q_col].values - wp_df[qdes_col].values)
            row[f"joint_{i+1}_mean_abs_error_rad"] = float(np.mean(err))
        results.append(row)

    return pd.DataFrame(results)


def compute_dwell_ee_metrics(df):
    mask = get_dwell_mask(df)
    if mask is None or mask.sum() == 0:
        return None

    results = []
    dwell_df = df[mask].copy()

    for wp in sorted(dwell_df["wp_idx"].dropna().unique()):
        wp_df = dwell_df[dwell_df["wp_idx"] == wp].copy()
        e_norm = get_ee_error(wp_df)
        results.append({
            "wp_idx": int(wp),
            "ee_mean_error_m": float(np.mean(e_norm)),
            "ee_max_error_m": float(np.max(e_norm)),
        })

    return pd.DataFrame(results)


def save_trial_summary(df, out_dir, tag):
    joint_rmse = compute_joint_rmse(df)
    joint_max = compute_joint_max_abs_error(df)
    ee_metrics = compute_ee_metrics(df)

    dwell_joint_df = compute_dwell_joint_mean_error(df)
    dwell_ee_df = compute_dwell_ee_metrics(df)

    out_path = os.path.join(out_dir, f"trial_summary_{tag}.csv")

    rows = []

    for i in range(6):
        joint_name = f"joint_{i+1}"

        dwell_mean = np.nan
        if dwell_joint_df is not None and not dwell_joint_df.empty:
            dwell_col = f"{joint_name}_mean_abs_error_rad"
            if dwell_col in dwell_joint_df.columns:
                dwell_mean = float(dwell_joint_df[dwell_col].mean())

        dwell_ee_mean = np.nan
        dwell_ee_max = np.nan
        if dwell_ee_df is not None and not dwell_ee_df.empty:
            dwell_ee_mean = float(dwell_ee_df["ee_mean_error_m"].mean())
            dwell_ee_max = float(dwell_ee_df["ee_max_error_m"].max())

        rows.append({
            "trial": tag,
            "joint": joint_name,
            "rmse_rad": joint_rmse[joint_name],
            "max_abs_error_rad": joint_max[joint_name],
            "dwell_mean_error_rad": dwell_mean,
            "ee_rmse_m": ee_metrics["rmse_ee_m"],
            "ee_max_error_m": ee_metrics["max_ee_m"],
            "ee_mean_error_m": ee_metrics["mean_ee_m"],
            "dwell_ee_mean_error_m": dwell_ee_mean,
            "dwell_ee_max_error_m": dwell_ee_max,
        })

    summary_df = pd.DataFrame(rows)
    summary_df.to_csv(out_path, index=False)

    print(f"Resumen del trial guardado en: {out_path}")
    print(summary_df.to_string(index=False))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    csv_files = get_all_csvs()

    for csv_path in csv_files:
        tag = os.path.splitext(os.path.basename(csv_path))[0]

        print(f"\nLeyendo CSV: {csv_path}")
        df = pd.read_csv(csv_path)

        plot_joint_tracking(df, OUT_DIR, tag)
        plot_task_space(df, OUT_DIR, tag)
        plot_3d_path(df, OUT_DIR, tag)
        plot_ee_error(df, OUT_DIR, tag)
        plot_phase_portraits(df, OUT_DIR, tag)
        plot_joint_velocities(df, OUT_DIR, tag)

        save_trial_summary(df, OUT_DIR, tag)

    print(f"\nGráficas y métricas guardadas en: {OUT_DIR}")

if __name__ == "__main__":
    main()