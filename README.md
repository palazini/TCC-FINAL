# Monitoramento de Sistemas Elétricos com ESP32
Este projeto, apresentado como TCC no SENAI Jandira, combina sensores e o microcontrolador ESP32 para criar uma solução robusta de monitoramento de sistemas elétricos. O código está estruturado para leitura, tratamento e envio de dados relacionados a variáveis críticas, como tensão, corrente, temperatura e vibração, possibilitando uma manutenção preditiva eficiente.

Principais Funcionalidades
Leitura e Processamento de Sensores:

Tensão (ZMPT101B): Captura a tensão RMS do sistema.
Corrente (ACS712): Monitora a corrente elétrica com compensação de offset.
Temperatura e Umidade (DHT22): Lê dados ambientais para análise.
Vibração (ADXL335): Mede vibração nos eixos X, Y e Z e calcula frequências com base em cruzamentos por zero.
Transmissão de Dados:

Envio dos dados para a plataforma WEGnology via protocolo MQTT.
Feedbacks de status no terminal para depuração.
Arquitetura Modular:

Tarefas FreeRTOS para leitura contínua de sensores e processamento assíncrono.
Mutex para evitar conflitos na leitura do DHT22.
Estrutura do Código
Tarefas FreeRTOS:

adxl335_task(): Mede vibração e calcula frequências.
zmpt_task(): Calcula a tensão RMS em ciclos contínuos.
calcular_corrente_task(): Processa a corrente RMS.
dht22_task(): Atualiza temperatura e umidade periodicamente.
Funções Auxiliares:

mqtt_app_start(): Configura e conecta o MQTT com autenticação via WEGnology.
calculate_frequency(): Converte cruzamentos por zero em frequência (Hz).
read_voltage(): Lê valores do ZMPT101B para calcular tensão RMS.
Como Usar
Clone o repositório:
bash
Copiar código
git clone https://github.com/seuusuario/monitoramento-sistemas-eletricos.git  
Configure o ambiente ESP-IDF no VSCode.
Ajuste as macros no código para os pinos e offsets dos sensores conforme o seu hardware.
Compile e carregue o firmware no ESP32.
Configure as credenciais MQTT no mqtt_app_start() para integração com sua conta no WEGnology.
Hardware Utilizado
ESP32: Controle e processamento.
Sensores:
ZMPT101B para tensão.
ACS712 (5A) para corrente.
DHT22 para temperatura/umidade.
ADXL335 para vibração.
Contribuições
Contribuições são bem-vindas! Se você tem ideias ou melhorias, sinta-se à vontade para abrir uma issue ou enviar um pull request.

💡 Autor: Gabriel Soares Palazini
📚 Instituição: SENAI Jandira
📬 Contato: gabrielpalazini1@gmail.com
