# Grafico theta_1..6(t) e cadencia de amostragem a partir de log_joints.csv
# Uso:  python plot_joints.py            (le log_joints.csv na mesma pasta)
#       python plot_joints.py dados.csv  (outro ficheiro)
# Aceita tambem dados copiados da consola do Studio: linhas que nao sao numeros sao ignoradas.
import sys
import matplotlib.pyplot as plt

ficheiro = sys.argv[1] if len(sys.argv) > 1 else 'log_joints.csv'

t, ang = [], [[] for _ in range(6)]
with open(ficheiro) as f:
    for linha in f:
        partes = linha.strip().split(',')
        if len(partes) != 7:
            continue                      # cabecalho, mensagens da consola, linhas vazias
        try:
            valores = [float(p) for p in partes]
        except ValueError:
            continue
        t.append(valores[0])
        for j in range(6):
            ang[j].append(valores[j + 1])

if len(t) < 2:
    sys.exit(f'Nao encontrei dados validos em {ficheiro}')

# Cadencia de amostragem
dts = [b - a for a, b in zip(t, t[1:])]
duracao = t[-1] - t[0]
cadencia = (len(t) - 1) / duracao
print(f'amostras: {len(t)} | duracao: {duracao:.2f} s')
print(f'cadencia media: {cadencia:.1f} Hz')
print(f'intervalo medio: {1000*sum(dts)/len(dts):.2f} ms | min: {1000*min(dts):.2f} ms | max: {1000*max(dts):.2f} ms')
for j in range(6):
    print(f'J{j+1}: min {min(ang[j]):8.2f}  max {max(ang[j]):8.2f}  amplitude {max(ang[j]) - min(ang[j]):7.2f} graus')

# Figura: angulos em cima, intervalo entre amostras em baixo
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True,
                            gridspec_kw={'height_ratios': [3, 1]})
for j in range(6):
    ax1.plot(t, ang[j], label=f'$\\theta_{j+1}$', linewidth=1.2)
ax1.set_ylabel('ângulo (graus)')
fig.suptitle(f'Ângulos das juntas em modo manual — {len(t)} amostras, {cadencia:.0f} Hz')
ax1.legend(ncol=6, loc='lower center', bbox_to_anchor=(0.5, 1.0), fontsize=9, frameon=False)
ax1.grid(True, alpha=0.3)

ax2.plot(t[1:], [1000 * d for d in dts], linewidth=0.8, color='gray')
ax2.set_xlabel('tempo (s)')
ax2.set_ylabel('Δt (ms)')
ax2.ticklabel_format(axis='y', useOffset=False)
ax2.grid(True, alpha=0.3)

fig.tight_layout()
fig.savefig('log_joints.png', dpi=200)
print('grafico guardado em log_joints.png')
plt.show()