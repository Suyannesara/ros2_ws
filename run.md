Aqui vai uma sequência de comandos para você testar todo o fluxo, com explicação de cada um:

1. **Inicie o Gazebo com o seu mundo (incluindo o sensor):**

   ```bash
   ros2 launch landing_zone_system gazebo.launch.py
   ```

   – Abre o Gazebo, carrega o seu `grass.world` (ou `garden.world`) e “spawna” o sensor de câmera que publica em `/camera/image_raw`.

2. **Rode o nó de busca de zona:**

   ```bash
   ros2 run landing_zone_system landing_zone_node
   ```

   – Inicializa o nó ROS 2 que fica escutando `/camera/image_raw` e o tópico `/find`. Ainda não começa a varrer até você publicar no `/find`.

3. **Dispare o comando de “find” via terminal:**

   ```bash
   ros2 topic pub /find std_msgs/msg/Empty "{}"
   ```

   – Publica uma mensagem vazia em `/find`, fazendo o seu nó iniciar a varredura dos blocos. Você verá no log do nó a mensagem “Find command received…” e, em seguida, os retângulos verdes na imagem.

4. **Verifique as coordenadas publicadas:**

   ```bash
   ros2 topic echo /zoneTopic geometry_msgs/msg/Point
   ```

   – Assim que a busca terminar, o nó publica o ponto central da zona encontrada em `/zoneTopic`. Este comando mostra no terminal `x` e `y` inteiros dessa zona.

5. **Visualize o vídeo anotado em tempo real:**

   * Com **rqt\_image\_view**:

     ```bash
     ros2 run rqt_image_view rqt_image_view
     ```

     Depois, no dropdown de tópicos, selecione `/annotated_image`.
   * (Opcional) Com **RViz2**:

     ```bash
     ros2 run rviz2 rviz2
     ```

     Em RViz, adicione um display do tipo **Image** e aponte para `/annotated_image`.

---

#### Fluxo resumido

1. `ros2 launch …gazebo.launch.py` → Gazebo + câmera virtual
2. `ros2 run …landing_zone_node` → nó pronto, aguardando `/find`
3. `ros2 topic pub /find …` → dispara a busca e marca blocos
4. `ros2 topic echo /zoneTopic …` → vê o resultado numérico
5. `rqt_image_view` ou `rviz2` → acompanha o vídeo anotado

Com esses cinco passos você consegue testar toda a cadeia: do Gazebo ao seu nó até a visualização e leitura das coordenadas.
