# **Introdução**

O projeto apresentado consiste na implementação de um **pêndulo invertido**, desenvolvido como parte das atividades da disciplina **Modelagem e Controle de Sistemas II**. A estrutura base foi adaptada a partir de uma impressora comum, aproveitando seu motor embutido de **24V** para a movimentação do trilho. Além da impressora, foram utilizados os seguintes componentes:

| Componente                                      | Descrição                                 | Datasheet                                                                                                                                                                                                           |
| ----------------------------------------------- | ----------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ESP32                                           | Microcontrolador                          | [📄](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)                                                                                                                            |
| 2 Sensores de distância VL53L0X                 | Sensor de distância (posição do carrinho) | [📄](https://www.alldatasheet.com/view.jsp?Searchword=Vl53l0x%20Datasheet&gad_source=1&gad_campaignid=1432848463&gclid=CjwKCAjw3MXBBhAzEiwA0vLXQTzKcui1WLHXg-_vCA1itTCsSSLOXApv7Bhh_TEmkd0yjqiV-MBufRoCYBwQAvD_BwE) |
| Encoder Incremental 360 AB PNP (F56)            | Medição do ângulo                         | [📄](https://www.alldatasheet.com/view.jsp?Searchword=Mpu-6050%20datasheet&gad_source=1&gad_campaignid=163458844&gclid=CjwKCAjw3MXBBhAzEiwA0vLXQTH4CT-uLhW6-a2hkWFem5TBKgU2mwys2hFuboTLkVvxGFpHKglb2RoCXcMQAvD_BwE) |
| Driver de motor BTS7960                         | Driver para motor DC                      | [📄](https://www.alldatasheet.com/view.jsp?Searchword=Bts7960%20datasheet&gad_source=1&gad_campaignid=145732807&gclid=CjwKCAjwprjDBhBTEiwA1m1d0hARd2jc2kv0Bl5XEZjIOlE777TRUl6Reo8d-SP2JrrT4wIWn1QZaRoC7yUQAvD_BwE)  |
| Fonte 24V Bivolt - 10A - 240W                   | Alimentação do motor                      |                                                                                                                                                                                                                     |
| Motor do Carro de Impressão da Impressora Epson | Motor 24V                                 |                                                                                                                                                                                                                     |

A combinação desses componentes permite a estabilização do pêndulo invertido por meio de estratégias de controle em tempo real, explorando conceitos teóricos aplicados na disciplina que serão apresentados a seguir.
# **Objetivos do Projeto**

O principal objetivo deste projeto é **estabilizar um pêndulo invertido dentro de uma região linearizada**, mantendo sua oscilação dentro de um limite de **±10°** em relação ao ponto de equilíbrio vertical. Para isso, serão utilizadas **técnicas de identificação de sistemas**, especificamente modelos **ARX** (Auto Regressive with eXogenous input), **ARMAX** (Auto Regressive Moving Average with eXogenous input), Alocação de polos e LQR (Regulador Linear Quadrático) para:

1. **Identificar dinamicamente o sistema** a partir de dados experimentais (ARX / ARMAX), ou obter um modelo matemático que represente adequadamente o comportamento do pêndulo (Alocação de polos / LQR).
2. **Projetar uma estratégia de controle** com base no modelo estimado, garantindo estabilidade dentro da faixa linear.
3. **Validar experimentalmente** o desempenho do controlador, analisando:
	- Tempo de estabilização.
	- Robustez a perturbações externas.
	- Limitações do modelo linearizado (considerando a restrição de ±10°).
4. **Comparar a eficiência** dos modelos **ARX** e **ARMAX** na representação do sistema.

# **Código**
## Controle do Pendulo
A partir dos códigos de testes dos sensores [Ponte H BTS7960](Manual/Teste_Sensores/Ponte_H_BTS7960.ino), [Sensor do Angulo Encoder](Manual/Teste_Sensores/Sensor_do_Angulo_Encoder.ino) e [Sensores de distancia VL53L0X](Manual/Teste_Sensores/Sensores_de_distancia_VL53L0X.ino) criamos um código completo de captura de variáveis do modelo e utilizando de referencia o modelo matemático do [Control Tutorials for MATLAB & SIMULINK](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling) e dos parâmetros LQR do repositório no GitHub que usamos de referencia e a partir dessas variáveis criamos o [Controle Pendulo](Manual/Controle_Pendulo.ino) e conferíamos os resultados através do [coletor serial](Manual/coletor_serial.py).

# Referências
KISHAN, I. **Inverted Pendulum**. GitHub, [S. l.], 2025. Disponível em: [https://github.com/imkishan96/Inverted_Pendulum/tree/master](https://github.com/imkishan96/Inverted_Pendulum/tree/master). Acesso em: 3 jun. 2025.

UNIVERSITY OF MICHIGAN. **Control Tutorials for MATLAB and Simulink: Inverted Pendulum - System Modeling**. [S. l.], 2025. Disponível em: [http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum§ion=SystemModeling](http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum%C2%A7ion=SystemModeling). Acesso em: 3 jun. 2025.
