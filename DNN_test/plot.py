import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ----------------------------- CONFIG -----------------------------
# csv_path = "/home/priyankan/Desktop/arm_bot/src/scripts/Joint_states/path_601_joint_states.csv"
# csv_path = "/home/priyankan/Downloads/path_451-600/path_564_joint_states_1.csv"
csv_path = "/home/priyankan/Desktop/FYP_DNN_CTC/Data/path_461_joint_states.csv"
# -----------------------------------------------------------------


def normalize_name(name: str) -> str:
    # Make matching robust: "tau 3", "tau_3", "Tau3" -> "tau3"
    return "".join(ch for ch in str(name).lower() if ch.isalnum())


def load_row_wise_csv(path: str) -> dict:
    """
    CSV format expected:
    row 0: t, t0, t1, ...
    row 1: dp1, ...
    row 2: dp2, ...
    ...
    Returns a dict of {normalized_name: numpy_array_of_values}
    """
    df = pd.read_csv(path, header=None, dtype=str)

    series = {}
    for i in range(df.shape[0]):
        raw_name = df.iat[i, 0]
        if pd.isna(raw_name):
            continue

        key = normalize_name(raw_name)
        row_vals = pd.to_numeric(df.iloc[i, 1:], errors="coerce").to_numpy(dtype=float)
        series[key] = row_vals

    return series


def get_series(series_map: dict, name: str) -> np.ndarray:
    key = normalize_name(name)
    if key not in series_map:
        available = ", ".join(sorted(series_map.keys()))
        raise KeyError(f"Series '{name}' not found. Available series: {available}")
    return series_map[key]


def plot_group(ax, t, ys, labels, title, y_label):
    for y, lbl in zip(ys, labels):
        # Handle any NaNs safely
        mask = np.isfinite(t) & np.isfinite(y)
        ax.plot(t[mask], y[mask], label=lbl, linewidth=1.8)

    ax.set_title(title)
    ax.set_xlabel("t")
    ax.set_ylabel(y_label)
    ax.grid(True, alpha=0.35)
    ax.legend()


def main():
    data = load_row_wise_csv(csv_path)

    t = get_series(data, "t")

    dp1 = get_series(data, "dp1")
    dp2 = get_series(data, "dp2")
    dp3 = get_series(data, "dp3")

    dv1 = get_series(data, "dv1")
    dv2 = get_series(data, "dv2")
    dv3 = get_series(data, "dv3")

    da1 = get_series(data, "da1")
    da2 = get_series(data, "da2")
    da3 = get_series(data, "da3")

    tau1 = get_series(data, "tau1")
    tau2 = get_series(data, "tau2")
    tau3 = get_series(data, "tau3")  # also matches "tau 3"

    # Make all series same length (safety for uneven rows)
    lengths = [
        len(t), len(dp1), len(dp2), len(dp3),
        len(dv1), len(dv2), len(dv3),
        len(da1), len(da2), len(da3),
        len(tau1), len(tau2), len(tau3)
    ]
    n = min(lengths)
    t = t[:n]
    dp1, dp2, dp3 = dp1[:n], dp2[:n], dp3[:n]
    dv1, dv2, dv3 = dv1[:n], dv2[:n], dv3[:n]
    da1, da2, da3 = da1[:n], da2[:n], da3[:n]
    tau1, tau2, tau3 = tau1[:n], tau2[:n], tau3[:n]

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    axes = axes.ravel()

    plot_group(
        axes[0], t,
        [dp1, dp2, dp3],
        ["dp1", "dp2", "dp3"],
        "Plot 1: t vs dp1, dp2, dp3",
        "dp"
    )

    plot_group(
        axes[1], t,
        [dv1, dv2, dv3],
        ["dv1", "dv2", "dv3"],
        "Plot 2: t vs dv1, dv2, dv3",
        "dv"
    )

    plot_group(
        axes[3], t,
        [da1, da2, da3],
        ["da1", "da2", "da3"],
        "Plot 3: t vs da1, da2, da3",
        "da"
    )

    plot_group(
        axes[2], t,
        [tau1, tau2, tau3],
        ["tau1", "tau2", "tau3"],
        "Plot 4: t vs tau1, tau2, tau3",
        "tau"
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()