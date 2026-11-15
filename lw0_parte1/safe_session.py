# LW0 Parte 1 - item 6: safe_session.py
# Uma funcao que deixa o robo pronto e SEGURO. Todos os scripts seguintes comecam por ela:
#     from safe_session import safe_session
#     arm = safe_session('192.168.1.xxx')
import time
from xarm.wrapper import XArmAPI

ROBOT_IP = '192.168.1.189'   # <-- mudar

# Limites da politica de seguranca (§3 do enunciado)
VEL_MAX_JUNTAS = 54          # graus/s  (30% de 180 graus/s)
VEL_MAX_TCP = 150            # mm/s

# Fronteira da TCP da vossa bancada: [x_max, x_min, y_max, y_min, z_max, z_min] em mm.
# Usem os valores que leram no ex2 (Tabela 1). Se deixarem None, mantem-se a fronteira
# que o laboratorio ja tem configurada no controlador (a opcao mais segura).
FRONTEIRA = None


def ok(code, o_que):
    """Para o programa se o robo devolver um codigo diferente de 0 (erro).
    Algumas funcoes do SDK 1.14.11 (ex.: set_fence_mode) devolvem o codigo
    dentro de uma lista, como [0]; nesse caso usa-se o primeiro elemento."""
    if isinstance(code, (list, tuple)):
        code = code[0] if code else -1
    if code != 0:
        raise RuntimeError(f'{o_que} falhou com code={code}')


def safe_session(ip, fronteira=FRONTEIRA):
    """Liga ao robo, limpa erros, ativa o movimento, poe modo 0 / estado 0,
    aplica o modo reduzido com os limites do §3 e uma fronteira da TCP.
    Devolve o objeto XArmAPI pronto a usar."""
    # 1. Ligar (mm e graus)
    arm = XArmAPI(ip, is_radian=False)

    # 2. Limpar avisos e erros
    ok(arm.clean_warn(), 'clean_warn')
    ok(arm.clean_error(), 'clean_error')

    # 3. Ativar o movimento (servos ligados)
    ok(arm.motion_enable(enable=True), 'motion_enable')

    # 4. Modo 0 (controlo de posicao) e estado 0 (pronto)
    ok(arm.set_mode(0), 'set_mode(0)')
    ok(arm.set_state(0), 'set_state(0)')

    # 5. Fronteira da TCP: a indicada, ou a que ja esta no controlador
    if fronteira is None:
        code, states = arm.get_reduced_states()
        ok(code, 'get_reduced_states')
        fronteira = list(states[1])
    if len(fronteira) != 6:
        raise ValueError('fronteira tem de ser [x_max, x_min, y_max, y_min, z_max, z_min]')
    ok(arm.set_reduced_tcp_boundary(fronteira), 'set_reduced_tcp_boundary')

    # 6. Limites de velocidade do modo reduzido
    ok(arm.set_reduced_max_joint_speed(VEL_MAX_JUNTAS), 'set_reduced_max_joint_speed')
    ok(arm.set_reduced_max_tcp_speed(VEL_MAX_TCP), 'set_reduced_max_tcp_speed')

    # 7. Ligar o modo reduzido (so depois disto os valores acima ficam ativos)
    ok(arm.set_reduced_mode(True), 'set_reduced_mode')
    ok(arm.set_fence_mode(True), 'set_fence_mode')   # fronteira de seguranca ligada

    # 8. Repor modo/estado (algumas definicoes deixam o braco parado)
    ok(arm.set_mode(0), 'set_mode(0)')
    ok(arm.set_state(0), 'set_state(0)')
    time.sleep(0.5)

    # 9. Verificar que ficou mesmo aplicado
    code, states = arm.get_reduced_states()
    ok(code, 'get_reduced_states')
    if not states[0]:
        raise RuntimeError('o modo reduzido NAO ficou ligado - nao usem o robo')
    print('safe_session: modo reduzido ligado')
    print('  fronteira TCP [xmax,xmin,ymax,ymin,zmax,zmin]:', states[1])
    print('  vel. max TCP (mm/s):', states[2], '| vel. max juntas (graus/s):', states[3])
    return arm


if __name__ == '__main__':
    arm = safe_session(ROBOT_IP)
    code, ang = arm.get_servo_angle(is_real=True)
    ok(code, 'get_servo_angle')
    print('pronto. angulos atuais:', [round(a, 2) for a in ang[:6]])
    arm.disconnect()