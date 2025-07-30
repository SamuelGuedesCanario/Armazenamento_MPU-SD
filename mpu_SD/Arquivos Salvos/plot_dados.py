# Importação das bibliotecas necessárias
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

dados_sensores_csv = 'MPU_data1.csv'

# Carrega os dados do arquivo CSV para um DataFrame do pandas
try:
    df = pd.read_csv(dados_sensores_csv)

    print("Leitura do arquivo CSV e início da visualização com Matplotlib...")

    # Define a coluna 'numero_amostra' como o índice do DataFrame para facilitar a plotagem
    df = df.set_index('numero_amostra')

    # Define as colunas que serão plotadas
    colunas_sensores = ['accel_x', 'accel_y', 'accel_z', 'giro_x', 'giro_y', 'giro_z']

    # Cria uma figura e um conjunto de subplots com matplotlib
    # Teremos 3 linhas e 2 colunas de gráficos
    fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(15, 12))
    fig.suptitle('Visualização dos Dados dos Sensores ao Longo do Tempo', fontsize=16)

    # Aplana o array de eixos para facilitar a iteração
    axes = axes.flatten()

    # Itera sobre cada coluna de sensor e plota em um subplot diferente
    for i, coluna in enumerate(colunas_sensores):
        axes[i].plot(df.index, df[coluna], label=coluna)
        axes[i].set_title(f'Leituras do {coluna.replace("_", " ").upper()}')
        axes[i].set_xlabel('Número da Amostra')
        axes[i].set_ylabel('Valor')
        axes[i].legend()
        axes[i].grid(True)

    # Ajusta o layout para evitar sobreposição de títulos
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

    print("\nGráficos individuais gerados com sucesso utilizando Matplotlib.")
    print("\n" + "="*50 + "\n")

except FileNotFoundError:
    print(f"Erro: O arquivo '{dados_sensores_csv}' não foi encontrado.")
    print("Por favor, certifique-se de que o arquivo está no ambiente do Google Colab.")