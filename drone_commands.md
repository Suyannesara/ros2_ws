Ótimo! Como você está usando **Gazebo Classic + PX4 com o drone `iris_depth_camera`**, e já tem o `micrortps_agent` rodando, você pode comandar o drone pelo terminal com mensagens **ROS 2** via `ros2 topic pub`.

Aqui vão os passos **mínimos e diretos** para:


### PRIMEIRO RODE O MIRORTPS EM UM TERMINAL
```bash
ros2 run px4_ros_com micrortps_agent -t UDP
```

---
EM OUTRO TERMINAL:


### 🔁 Extra: Armar o drone (se necessário)

```bash
ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{
  command: 400,
  param1: 2.0,
  target_system: 1,
  target_component: 1,
  source_system: 1,
  source_component: 1,
  from_external: true
}"
```

### ✅ 1. **Decolar (Takeoff)**

Publique uma mensagem para `VehicleCommand` com o modo decolar:

```bash
ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{
  param1: 2.0,
  command: 22,
  target_system: 1,
  target_component: 1,
  source_system: 1,
  source_component: 1,
  from_external: true
}"
```

* `command: 22` → Takeoff (`VEHICLE_CMD_NAV_TAKEOFF`)
* `param1: 1.0` → Altura de decolagem (metros)

---

### ✅ 2. **Ir para frente (ou em outra direção)**

Para movimento manual, envie velocidades com `OffboardControlMode` + `VehicleLocalPositionSetpoint`.

⚠️ Mas **antes disso**, é necessário ativar o **modo offboard**:

#### (a) Ativar modo offboard:

```bash
ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{
  command: 176,
  param1: 1.0,
  target_system: 1,
  target_component: 1,
  source_system: 1,
  source_component: 1,
  from_external: true
}"
```

#### (b) Enviar controle de velocidade para frente:

```bash
ros2 topic pub /fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint "{
  velocity: [1.0, 0.0, 0.0]
}"
```

* `velocity: [1.0, 0.0, 0.0]` → 1 m/s para frente (eixo X)
* `[-1.0, 0.0, 0.0]` → Para trás

🔁 Para manter movimento, o comando precisa ser reenviado em \~20 Hz (use `--rate 20` no pub).

Exemplo com repetição:

```bash
ros2 topic pub --rate 20 /fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint "{velocity: [1.0, 0.0, 0.0]}"
```

---

### ✅ 3. **Pousar**

```bash
ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{
  command: 21,
  target_system: 1,
  target_component: 1,
  source_system: 1,
  source_component: 1,
  from_external: true
}"
```

* `command: 21` → `VEHICLE_CMD_NAV_LAND`

---

Se quiser, posso te passar um script `.sh` para automatizar esse fluxo. Deseja isso?


ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand '{param1: 1.0, command: 400, target_system: 1, target_component: 1, source_system: 1, source_component: 1}'

Sim, exatamente — **o `micrortps_agent`** que você mencionou é parte da **ponte uXRCE-DDS**, usada para comunicação entre o PX4 e o ROS 2 quando o PX4 está rodando em hardware real ou em SITL (como no Gazebo). Aqui vão os pontos principais para garantir que está tudo certo:

---

### ✅ **O que é o `micrortps_agent`?**

É o agente da ponte **microRTPS** que faz a interface entre o PX4 (usando FastRTPS no firmware) e o **ROS 2**, utilizando o protocolo DDS (Data Distribution Service). Ele é **necessário quando você quer se comunicar com o PX4 via ROS 2**.

---

### 🛠️ **Verificações e Passos para rodar o `micrortps_agent`:**

1. **Você clonou e compilou os repositórios corretamente?**

   * Clonar os pacotes:

     ```bash
     cd ~/ros2_ws/src
     git clone https://github.com/PX4/px4_msgs.git
     git clone https://github.com/PX4/px4_ros_com.git
     ```
   * Compilar:

     ```bash
     cd ~/ros2_ws
     source /opt/ros/humble/setup.bash
     colcon build
     source install/setup.bash
     ```

2. **Verificar se o executável existe:**
   Após compilar, rode:

   ```bash
   ros2 run px4_ros_com micrortps_agent
   ```

   Se aparecer "No executable found", pode ser que:

   * O build falhou (veja se `colcon build` terminou com sucesso).
   * O pacote `px4_ros_com` não está com os executáveis definidos corretamente no `CMakeLists.txt`.
   * O `micrortps_agent` não é um executável próprio do pacote ROS, mas sim do PX4/FastDDS.

---

### ⚠️ **Alternativa mais recomendada para você (usando Gazebo com SITL)**

Se você está **simulando no Gazebo Classic com PX4 SITL**, **não precisa do micrortps\_agent**. O próprio PX4 já publica tópicos ROS 2 diretamente via um **bridge interno**, se o mundo foi iniciado com o comando:

```bash
make px4_sitl gazebo-classic_iris__baylands
```

Nesse caso, o **PX4 usa um bridge interno com FastDDS** — e você deve usar o `px4_msgs` no ROS 2 para se comunicar.

---

### ✈️ Comandos básicos via ROS 2 para controle:

Se a ponte está funcionando e o drone PX4 está rodando no Gazebo, **você pode publicar comandos diretamente**. Aqui estão exemplos:

1. **Armar o drone (necessário antes de qualquer voo):**

   ```bash
   ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand '{param1: 1.0, command: 400, target_system: 1, target_component: 1, source_system: 1, source_component: 1}'
   ```

2. **Desarmar:**

   ```bash
   ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand '{param1: 0.0, command: 400, target_system: 1, target_component: 1, source_system: 1, source_component: 1}'
   ```

3. **Takeoff:**

   ```bash
   ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand '{command: 22, param7: 5.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1}'
   ```

4. **Ir para frente (movimento manual com setpoint velocity ou trajectory setpoint)**
   Pode ser mais fácil criar um script para publicar em `/fmu/in/trajectory_setpoint`.

5. **Pousar:**

   ```bash
   ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand '{command: 21, target_system: 1, target_component: 1, source_system: 1, source_component: 1}'
   ```

---

Se quiser, posso te ajudar a criar um script simples Python para enviar esses comandos, ou fazer isso direto no terminal com `ros2 topic pub`.

Quer seguir com os comandos via terminal ou script Python?
