import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple
import csv
from ament_index_python.packages import get_package_share_directory
import os

# ------------------------- Параметры робота -------------------------
L1 = 0.35  # м
L2 = 0.25  # м

# ------------------------- Dataclasses -------------------------
@dataclass
class TrajectoryPoint:
    t: float
    q1: float
    q2: float

@dataclass
class Segment:
    start: Tuple[float, float]
    end: Tuple[float, float]
    duration: float

# ------------------------- Функция IK -------------------------
def ik(x: float, y: float, q_prev: Tuple[float, float]) -> Tuple[float, float]:
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    D = np.clip(D, -1.0, 1.0)
    
    q2_candidates = [
        np.arctan2(np.sqrt(1 - D**2), D),
        np.arctan2(-np.sqrt(1 - D**2), D)
    ]
    q2_new = min(q2_candidates, key=lambda q: abs(q - q_prev[1]))
    q1_new = np.arctan2(y, x) - np.arctan2(L2 * np.sin(q2_new), L1 + L2 * np.cos(q2_new))
    
    return q1_new, q2_new

# ------------------------- Генерация траектории -------------------------

def generate_trajectory(segments: List[Segment], dt: float = 0.001) -> List[TrajectoryPoint]:
    points: List[TrajectoryPoint] = []
    current_t = 0.0
    
    q1_urdf_start = 0.649437
    q2_urdf_start = 1.036608
    
    # Инициализация: первая точка первого сегмента
    start_x, start_y = segments[0].start
    q1, q2 = ik(start_x, start_y, (q1_urdf_start, q2_urdf_start)) # <-- ИЗМЕНЕНО ЗДЕСЬ
    
    for seg_idx, seg in enumerate(segments):
        x_start, y_start = seg.start
        x_end, y_end = seg.end
        T = seg.duration
        
        L = np.hypot(x_end - x_start, y_end - y_start)
        v = L / T if T > 0 else 0.0
        
        if L > 1e-12:
            ux = (x_end - x_start) / L
            uy = (y_end - y_start) / L
        else:
            ux = uy = 0.0
            
        xdot = v * ux
        ydot = v * uy
        
        n_steps = max(1, int(round(T / dt)))
        t_seg = np.linspace(0, T, n_steps + 1)
        
        for i, tau in enumerate(t_seg):
            t_abs = current_t + tau
            
            x = x_start + xdot * tau
            y = y_start + ydot * tau
            
            # ВАЖНО: передаём текущие q1, q2 как предыдущее состояние
            q1, q2 = ik(x, y, (q1, q2))
            
            points.append(TrajectoryPoint(
                t=t_abs,
                q1=q1,
                q2=q2
            ))
            
        current_t += T
    
    # Развёртка углов — убираем скачки ±2π (ваша оригинальная логика)
    q1_arr = np.unwrap([p.q1 for p in points])
    q2_arr = np.unwrap([p.q2 for p in points])
    
    for i in range(len(points)):
        points[i] = TrajectoryPoint(t=points[i].t, q1=float(q1_arr[i]), q2=float(q2_arr[i]))
    
    return points
# ------------------------- Точки контура пятиконечной звезды -------------------------
# R_out = 0.40 м (вершины), R_in = 0.20 м (впадины)
# Начало с верхней вершины, обход по часовой стрелке
STAR_POINTS: List[Tuple[float, float]] = [
    ( 0.10,  0.10),  # 0: верх
    ( 0.20,  0.20),  # 1
    ( 0.10,  0.30),  # 2: право
    ( 0.25,  0.40),  # 3S
    ( 0.40,  0.30),  # 4: низ-право
    ( 0.30,  0.20),  # 5: низ
    ( 0.40,  0.10),  # 6: низ-лево
    (0.10,  0.10),  # 7
]
# ------------------------- Основная программа -------------------------
def main():
    seg_duration = 3.0  # секунд на каждый сегмент
    
    
    segments: List[Segment] = []
    for i in range(len(STAR_POINTS)):
        p_start = STAR_POINTS[i]
        p_end = STAR_POINTS[(i + 1) % len(STAR_POINTS)]
        segments.append(Segment(p_start, p_end, seg_duration))
    
    print(f"Сформировано сегментов: {len(segments)}")
    
    dt = 0.001
    points = generate_trajectory(segments, dt)
    
    time = [p.t for p in points]
    q1 = [p.q1 for p in points]
    q2 = [p.q2 for p in points]
    
    print(f'\nНачальные углы: q1={q1[0]:.4f}, q2={q2[0]:.4f}')
    print(f'Конечные углы:  q1={q1[-1]:.4f}, q2={q2[-1]:.4f}')
    
    dq1 = np.abs(np.diff(q1))
    dq2 = np.abs(np.diff(q2))
    print(f'Макс. скачок q1 между шагами: {dq1.max():.6f} рад')
    print(f'Макс. скачок q2 между шагами: {dq2.max():.6f} рад')
    
    # ═══════════════════════════════════════════════════════════════
    # СОХРАНЕНИЕ В CSV
    # ═══════════════════════════════════════════════════════════════
    pkg_scara_sim = get_package_share_directory('scara_sim')
    data_dir = os.path.join(pkg_scara_sim, 'data')
    os.makedirs(data_dir, exist_ok=True)
    
    file_path = os.path.join(data_dir, 'trajectory_angles.csv')
    
    with open(file_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'q1', 'q2'])
        for p in points:
            writer.writerow([p.t, p.q1, p.q2])
            
    print(f"\nCSV сохранён в {file_path}")
    print(f"Всего точек: {len(points)}")
    print(f"Длительность: {points[-1].t:.2f} с")
    
    # ═══════════════════════════════════════════════════════════════
    # ГРАФИКИ
    # ═══════════════════════════════════════════════════════════════
    fig = plt.figure(figsize=(14, 10))
    
    # 1) Траектория в пространстве (x, y)
    ax_xy = fig.add_subplot(2, 2, 1)
    x_traj = [L1*np.cos(q1[i]) + L2*np.cos(q1[i]+q2[i]) for i in range(len(points))]
    y_traj = [L1*np.sin(q1[i]) + L2*np.sin(q1[i]+q2[i]) for i in range(len(points))]
    ax_xy.plot(x_traj, y_traj, 'b-', linewidth=1.2, label='траектория')
    sx = [p[0] for p in STAR_POINTS] + [STAR_POINTS[0][0]]
    sy = [p[1] for p in STAR_POINTS] + [STAR_POINTS[0][1]]
    ax_xy.plot(sx, sy, 'ro', markersize=6, label='вершины звезды')
    ax_xy.plot(sx, sy, 'r--', alpha=0.4)
    ax_xy.set_aspect('equal')
    ax_xy.set_xlabel('x, м')
    ax_xy.set_ylabel('y, м')
    ax_xy.set_title('Контур пятиконечной звезды')
    ax_xy.grid(True)
    ax_xy.legend()
    
    # 2) q1(t)
    ax_q1 = fig.add_subplot(2, 2, 2)
    ax_q1.plot(time, q1)
    ax_q1.set_xlabel('Время, с')
    ax_q1.set_ylabel('q1, рад')
    ax_q1.set_title('Эталонный угол q1')
    ax_q1.grid(True)
    
    # 3) q2(t)
    ax_q2 = fig.add_subplot(2, 2, 3)
    ax_q2.plot(time, q2)
    ax_q2.set_xlabel('Время, с')
    ax_q2.set_ylabel('q2, рад')
    ax_q2.set_title('Эталонный угол q2')
    ax_q2.grid(True)
    
    # 4) Фазовый портрет
    ax_phase = fig.add_subplot(2, 2, 4)
    ax_phase.plot(q1, q2)
    ax_phase.set_xlabel('q1, рад')
    ax_phase.set_ylabel('q2, рад')
    ax_phase.set_title('Фазовый портрет (q1, q2)')
    ax_phase.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
