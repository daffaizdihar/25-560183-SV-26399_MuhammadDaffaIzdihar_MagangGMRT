# Muhammad Daffa Izdihar (25/560183/SV/26399)

import math
import numpy as np
import matplotlib.pyplot as plt

# Panjang segmen: (Femur=83, Tibia=99, Coxa=0) in cm
FEMUR = 83.0
TIBIA = 99.0

def rot_matrix(deg):
    rad = math.radians(deg)
    c, s = math.cos(rad), math.sin(rad)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0]
    ])

def trans_matrix(tx, ty):
    return np.array([
        [1.0, 0.0, tx],
        [0.0, 1.0, ty],
        [0.0, 0.0, 1.0]
    ])

def calc_fk(theta1, theta2):
    # Homogeneous matrix dari base (0) hingga ujung tibia (4)
    H1_0 = rot_matrix(theta1)
    H2_1 = trans_matrix(FEMUR, 0.0)
    H3_2 = rot_matrix(theta2)
    H4_3 = trans_matrix(TIBIA, 0.0)

    H2_0 = H1_0 @ H2_1
    H4_0 = H2_0 @ H3_2 @ H4_3

    # Ekstrak (x,y)
    knee = (H2_0.item(0,2), H2_0.item(1,2))
    foot = (H4_0.item(0,2), H4_0.item(1,2))
    return knee, foot, H4_0

def in_workspace(x, y): # Cek apakah jarak target dalam jangkauan
    if not (math.isfinite(x) and math.isfinite(y)):
        print("Input invalid. Koordinat harus berupa angka normal.")
        return
    r = math.hypot(x,y)
    r_min = abs(FEMUR - TIBIA)
    r_max = FEMUR + TIBIA

    if r==0:
        print("Warning: Target tepat di titik (0,0), sudut tidak terdefinisi.")
        return False
    if r>r_max: 
        print(f"Out of Workspace: target terlalu jauh (jarak = {r:.2f} > maks {r_max:.2f}).")
        return False
    if r<r_min:
        print(f"Out of Workspace: target terlalu dekat (jarak = {r:.2f} < min {r_min:.2f}).")
        return False
    return True

def calc_ik(x, y):
    if not in_workspace(x, y):
        return None

    # Hitung theta2 dengan aturan kosinus (dibatasi [-1,1])
    cos_t2 = (x**2 + y**2 - FEMUR**2 - TIBIA**2) / (2.0 * FEMUR * TIBIA)
    cos_t2 = max(-1.0, min(1.0, cos_t2))
    t2 = math.acos(cos_t2)

    # Pakai atan2 agat kuadran theta1 tepat
    t1 = math.atan2(y, x) - math.atan2(TIBIA*math.sin(t2), FEMUR+TIBIA*math.cos(t2))
    return math.degrees(t1), math.degrees(t2)

def draw_leg(knee, foot, title, save_name): # Visualisasi matplotlib
    # Bongkar koordinat knee dan foot
    kx, ky = knee
    fx, fy = foot
    xs = [0.0, kx, fx]
    ys = [0.0, ky, fy]

    # Konfigurasi visual sizing
    plt.figure(figsize=(7,7))
    plt.plot(xs, ys, marker="o", linewidth=3, markersize=8, label="Kaki Robot")

    # Labeling
    plt.text(3, -10, "Coxa (0,0)")
    plt.text(kx+4, ky+4, f"Lutut ({kx:.2f}, {ky:.2f})")
    plt.text(fx+4, fy+4, f"Ujung ({fx:.2f}, {fy:.2f})")

    # Konfigurasi visual sumbu
    limit = FEMUR+TIBIA+15
    plt.xlim(-limit, limit)
    plt.ylim(-limit, limit)
    plt.axhline(0, color="gray", linewidth=0.8)
    plt.axvline(0, color="gray", linewidth=0.8)
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.gca().set_aspect("equal")
    plt.xlabel("X (cm)")
    plt.ylabel("Y (cm)")
    plt.title(title)
    plt.legend()
    plt.savefig(save_name, dpi=200, bbox_inches="tight")
    print(f"Grafik berhasil disimpan ke {save_name}")
    plt.show()
    plt.close()

def fk_menu(): # Menu Forward Kinematics
    try: # Input sudut
        t1 = float(input("Masukkan sudut Servo 1 / theta1 (derajat): "))
        t2 = float(input("Masukkan sudut Servo 2 / theta2 (derajat): "))
    except ValueError:
        print("Input tidak valid. Harap masukkan angka.")
        return
    if not (math.isfinite(t1) and math.isfinite(t2)):
        print("Input invalid. Sudut harus berupa angka normal.")
        return
    knee, foot, H4_0 = calc_fk(t1, t2) # Akses fungsi (calculate) FK
    kx,ky = knee
    fx,fy = foot

    print("\n--- Hasil Forward Kinematics ---")
    print(f"Posisi Lutut       : ({kx:.2f}, {ky:.2f}) cm")
    print(f"Posisi End-Effector: ({fx:.2f}, {fy:.2f}) cm")
    print("Matriks H4_0:")
    print(np.round(H4_0, 4))
    draw_leg(knee, foot, f"Postur FK (theta1={t1} deg, theta2={t2} deg)", f"fk_{t1}_{t2}.png")

def ik_menu(): # Menu Inverse Kinematics
    try: # Input target
        x = float(input("Masukkan target X (cm): "))
        y = float(input("Masukkan target Y (cm): "))
    except ValueError:
        print("Input tidak valid. Harap masukkan angka.")
        return

    result = calc_ik(x,y) # Akses fungsi (calculate) IK
    if result is None:
        return
    t1,t2 = result
    knee,foot,_= calc_fk(t1,t2)
    fx,fy = foot

    print("\n--- Hasil Inverse Kinematics ---")
    print(f"theta1\t: {t1:.2f} derajat.")
    print(f"theta2\t: {t2:.2f} derajat.")
    selisih = math.hypot(fx-x, fy-y)
    print(f"Verifikasi FK -> ({fx:.2f}, {fy:.2f}) cm (selisih: {selisih:.2f} cm)")

    draw_leg(knee, foot, f"Postur IK Target ({x:.2f}, {y:.2f}) cm", f"ik_{x}_{y}.png")

def main():
    while True: 
        print("\n=== Kinematika Kaki Robot 2-DoF ===")
        print(f"Femur = {FEMUR:.0f} cm | Tibia = {TIBIA:.0f} cm| Coxa = 0 cm")
        print("1. Forward Kinematics (FK)")
        print("2. Inverse Kinematics (IK)")
        print("3. Keluar")

        choice = input("Pilih menu (1/2/3): ")
        if choice=="1": fk_menu()
        elif choice=="2": ik_menu()
        elif choice=="3":
            print("Program selesai.")
            break
        else: print("Pilihan tidak valid. Masukkan 1, 2, atau 3!")

if __name__ == "__main__":
    main()