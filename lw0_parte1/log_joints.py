# LW0 Parte 1 - item 5: log_joints.py
# Le os 6 angulos das juntas durante 30 s, o mais depressa possivel,
# enquanto um colega guia o braco a mao em modo manual.
# Resultado: log_joints.csv (tempo + J1..J6) e, se houver matplotlib, log_joints.png.
import time
import csv
from xarm.wrapper import XArmAPI

ROBOT_IP = '192.168.1.189'   # <-- mudar
DURACAO = 30                 # segundos
PRINT_CSV = False            # True para imprimir os dados na consola (util no Python IDE do Studio)

try:
    from safe_session import safe_session, ok
except ImportError:          # se safe_session.py nao estiver na mesma pasta
    from xarm.wrapper import XArmAPI
    def ok(code, o_que):
        if code != 0:
            raise RuntimeError(f'{o_que} falhou com code={code}')
    def safe_session(ip):
        arm = XArmAPI(ip, is_radian=False)
        arm.clean_warn(); arm.clean_error()
        ok(arm.motion_enable(enable=True), 'motion_enable')
        ok(arm.set_mode(0), 'set_mode(0)'); ok(arm.set_state(0), 'set_state(0)')
        return arm

arm = safe_session(ROBOT_IP)

# Modo manual (modo 2). A carga da TCP TEM de estar certa (0.28 kg com a garra).
ok(arm.set_mode(2), 'set_mode(2)')
ok(arm.set_state(0), 'set_state(0)')
for s in (3, 2, 1):
    print(f'a comecar em {s}...'); time.sleep(1)
print(f'A GRAVAR {DURACAO} s - guiem o braco a mao')

tempos, dados, falhas = [], [], 0
t0 = time.perf_counter()
try:
    while True:
        t = time.perf_counter() - t0
        if t >= DURACAO:
            break
        code, ang = arm.get_servo_angle(is_real=True)   # angulos MEDIDOS
        if code == 0:
            tempos.append(t)
            dados.append(ang[:6])
        else:
            falhas += 1
finally:
    # Sair sempre do modo manual, mesmo que algo corra mal
    arm.set_mode(0); arm.set_state(0)
    arm.disconnect()

# Cadencia de amostragem
n = len(tempos)
dts = [b - a for a, b in zip(tempos, tempos[1:])]
cadencia = (n - 1) / (tempos[-1] - tempos[0])
print(f'amostras: {n} | leituras falhadas: {falhas}')
print(f'cadencia media: {cadencia:.1f} Hz')
print(f'intervalo medio: {1000*sum(dts)/len(dts):.2f} ms | maximo: {1000*max(dts):.2f} ms')

# Guardar CSV
with open('log_joints.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t_s', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
    for t, a in zip(tempos, dados):
        w.writerow([f'{t:.4f}'] + [f'{x:.4f}' for x in a])
print('dados guardados em log_joints.csv')

if PRINT_CSV:
    print('t_s,J1,J2,J3,J4,J5,J6')
    for t, a in zip(tempos, dados):
        print(f'{t:.4f},' + ','.join(f'{x:.4f}' for x in a))

# Grafico theta_1..6(t)
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 5))
    for j in range(6):
        ax.plot(tempos, [a[j] for a in dados], label=f'J{j+1}')
    ax.set_xlabel('tempo (s)')
    ax.set_ylabel('angulo (graus)')
    ax.set_title(f'Angulos das juntas em modo manual ({cadencia:.0f} Hz)')
    ax.legend(ncol=6, loc='upper center')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig('log_joints.png', dpi=150)
    print('grafico guardado em log_joints.png')
except ImportError:
    print('matplotlib nao disponivel: facam o grafico depois com plot_joints.py')