"""Imprime a pose de link_eef relativamente a link_base, com 6 casas decimais.
Correr dentro do contentor, com a simulação ligada:   python3 ~/shared/ler_pose.py
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node('ler_pose')
buf = Buffer()
TransformListener(buf, node)

fim = time.time() + 5.0
while time.time() < fim:
    rclpy.spin_once(node, timeout_sec=0.1)
    if buf.can_transform('link_base', 'link_eef', Time()):
        t = buf.lookup_transform('link_base', 'link_eef', Time()).transform.translation
        print(f'x = {t.x:.6f}   y = {t.y:.6f}   z = {t.z:.6f}   (metros)')
        break
else:
    print('Não encontrei link_base -> link_eef. A simulação está ligada?')

node.destroy_node()
rclpy.shutdown()
