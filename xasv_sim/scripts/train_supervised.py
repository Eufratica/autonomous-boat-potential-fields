#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
train_supervised.py

Pré-treina uma MLP com base em um ou mais CSVs
gerados por hitl_pretrain_pointcloud_node.py.

Entrada (FEATURE_COLS):
    min_front, min_left, min_right,
    speed, yaw,
    dist_to_wp, bearing_to_wp

Saída (TARGET_COLS):
    rc_throttle_norm, rc_yaw_norm   (em [-1, 1])

Ideia HITL:
  - Damos MAIS peso na loss para amostras em que o humano realmente
    mexeu nos sticks (|rc| grande), e MENOS peso quando está neutro.
  - (Opcional) Data augmentation por simetria esquerda/direita.
  - Damos MAIS peso no erro de yaw do que no erro de throttle.
"""

import csv
import argparse
import glob
import os          # <--- add this
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader


FEATURE_COLS = [
    "min_front", "min_left", "min_right",
    "speed", "yaw",
    "dist_to_wp", "bearing_to_wp",
]

TARGET_COLS = [
    "rc_throttle_norm",
    "rc_yaw_norm",
]


# Base dir = xasv_sim (this script lives in xasv_sim/scripts/)
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT = os.path.join(
    BASE_DIR,
    "data",
    "xasv_huitl_policy",
    "trained_rc_policy.pt",
)

class HitlDataset(Dataset):
    def __init__(self, csv_paths, symmetry_aug=False):
        """
        csv_paths: lista de caminhos ou padrão glob já expandido.
        symmetry_aug:
          - se True, gera amostras espelhadas:
            * min_left <-> min_right
            * yaw -> -yaw
            * bearing_to_wp -> -bearing_to_wp
            * rc_yaw_norm -> -rc_yaw_norm
        """
        if isinstance(csv_paths, str):
            csv_paths = [csv_paths]

        self.symmetry_aug = symmetry_aug
        self.X = []
        self.Y = []
        total_files = 0

        for path in csv_paths:
            matched = glob.glob(path)
            if not matched:
                print(f"[HitlDataset] Aviso: nenhum arquivo casa com '{path}'")
                continue

            for csv_path in matched:
                total_files += 1
                local_count = 0
                with open(csv_path) as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        try:
                            x = [float(row[c]) for c in FEATURE_COLS]
                            y = [float(row[c]) for c in TARGET_COLS]
                        except (ValueError, KeyError):
                            continue

                        # amostra original
                        self.X.append(x)
                        self.Y.append(y)
                        local_count += 1

                        # --- data augmentation por simetria esquerda/direita ---
                        if self.symmetry_aug:
                            x_m = x.copy()
                            y_m = y.copy()

                            # indices: 0 front, 1 left, 2 right, 3 speed, 4 yaw, 5 dist, 6 bearing
                            x_m[1], x_m[2] = x_m[2], x_m[1]   # left <-> right
                            x_m[4] = -x_m[4]                  # yaw -> -yaw
                            x_m[6] = -x_m[6]                  # bearing -> -bearing

                            # targets: 0 thr, 1 yaw
                            y_m[1] = -y_m[1]                  # yaw action -> -yaw

                            self.X.append(x_m)
                            self.Y.append(y_m)

                print(f"[HitlDataset] Loaded {local_count} samples from {csv_path}")

        if not self.X:
            raise RuntimeError(
                f"Nenhuma amostra válida encontrada nos CSVs: {csv_paths}"
            )

        print(f"[HitlDataset] Total de arquivos usados: {total_files}")
        print(f"[HitlDataset] Total de amostras (incl. augmentation): {len(self.Y)}")

        self.X = torch.tensor(self.X, dtype=torch.float32)
        self.Y = torch.tensor(self.Y, dtype=torch.float32)

    def __len__(self):
        return len(self.Y)

    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]


class PolicyMLP(nn.Module):
    def __init__(self, in_dim, hidden=64, out_dim=2):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, out_dim)
        )

    def forward(self, x):
        return self.net(x)


def main():
    parser = argparse.ArgumentParser()
    # aceita 1 ou mais CSVs / padrões glob
    parser.add_argument(
        "--csv",
        nargs="+",
        required=True,
        help="um ou mais caminhos/padrões de CSV (ex: log1.csv log2.csv ou /tmp/xasv_hitl_pretrain/*.csv)",
    )
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch_size", type=int, default=64)
    parser.add_argument(
        "--out",
        default=DEFAULT_OUT,
        help=(
            "output .pt file for the trained policy "
            f"(default: {DEFAULT_OUT})"
        ),
    )
    parser.add_argument(
        "--symmetry_aug",
        action="store_true",
        help="ativa data augmentation por simetria esquerda/direita",
    )
    # pesos relativos para throttle e yaw na loss
    parser.add_argument(
        "--thr_loss_weight",
        type=float,
        default=0.5,
        help="peso da perda no canal de throttle (default=0.5)",
    )
    parser.add_argument(
        "--yaw_loss_weight",
        type=float,
        default=1.5,
        help="peso da perda no canal de yaw (default=1.5, mais importante)",
    )
    args = parser.parse_args()

    ds = HitlDataset(args.csv, symmetry_aug=args.symmetry_aug)
    dl = DataLoader(ds, batch_size=args.batch_size, shuffle=True)

    model = PolicyMLP(in_dim=len(FEATURE_COLS), out_dim=len(TARGET_COLS))
    # MSE por elemento, sem reduzir ainda (vamos pesar amostras e canais)
    criterion = nn.MSELoss(reduction="none")
    optim = torch.optim.Adam(model.parameters(), lr=1e-3)

    # quanto mais ação humana (|rc| grande), mais peso na loss
    action_weight_alpha = 5.0  # pode testar 2.0, 5.0, 10.0...

    w_thr = args.thr_loss_weight
    w_yaw = args.yaw_loss_weight
    print(f"[train] thr_loss_weight={w_thr}, yaw_loss_weight={w_yaw}")

    for epoch in range(args.epochs):
        model.train()
        total_loss = 0.0
        for X, Y in dl:
            pred = model(X)               # [B, 2]
            se = criterion(pred, Y)       # [B, 2] erro quadrático por saída

            # magnitude da ação (0 = neutro, até ~2 = thr=1,yaw=1)
            mag = torch.clamp(Y.abs().sum(dim=1), 0.0, 2.0) / 2.0  # [0,1]
            # peso por amostra: 1 (neutro) até 1+alpha (ação forte)
            w_sample = 1.0 + action_weight_alpha * mag             # [B]

            # se[:,0] = erro thr², se[:,1] = erro yaw²
            # combinamos com pesos diferentes para cada canal
            loss_per_sample = w_thr * se[:, 0] + w_yaw * se[:, 1]  # [B]

            # média ponderada entre amostras
            loss = (loss_per_sample * w_sample).mean()

            optim.zero_grad()
            loss.backward()
            optim.step()
            total_loss += loss.item() * X.size(0)

        avg_loss = total_loss / len(ds)
        print(f"Epoch {epoch+1}/{args.epochs} - weighted MSE={avg_loss:.6f}")

    # Ensure output directory exists
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    torch.save({
        "state_dict": model.state_dict(),
        "feature_cols": FEATURE_COLS,
        "target_cols": TARGET_COLS,
    }, args.out)
    print(f"Saved trained model to {args.out}")



if __name__ == "__main__":
    main()

