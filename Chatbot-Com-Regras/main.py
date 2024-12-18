import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re

intents = {
    "va para": r"(va para|dirija-se ao|me leve para|se dirija ate)\s+(\w+)",
}

def go_to_location(location):
    return f"O robô esta se dirigindo para a {location}."

def action_not_understood():
    return "Desculpe, não entendi o comando. Tente novamente."

actions = {
    "va para": go_to_location,
}

class ChatbotNode(Node):
    def __init__(self):
        super().__init__('chatbot_node')
        self.publisher_ = self.create_publisher(String, 'chatbot_feedback', 10)
        self.get_logger().info("Nó do chatbot iniciado. Digite comandos para o robô.")

    def process_command(self, command):
        for intent, pattern in intents.items():
            match = re.search(pattern, command, re.IGNORECASE)
            if match:
                location = match.group(2)
                if intent in actions:
                    response = actions[intent](location)
                    return response
        return action_not_understood()

    def run(self):
        try:
            while rclpy.ok():
                user_input = input("Digite um comando: ").strip()
                if user_input.lower() in ["sair", "exit", "quit"]:
                    self.get_logger().info("Encerrando o chatbot.")
                    break

                # Processamento do comando
                feedback = self.process_command(user_input)
                self.get_logger().info(feedback)

                # Publica feedback no tópico
                msg = String()
                msg.data = feedback
                self.publisher_.publish(msg)

        except KeyboardInterrupt:
            self.get_logger().info("Chatbot interrompido pelo usuario.")

def main(args=None):
    rclpy.init(args=args)
    chatbot = ChatbotNode()
    chatbot.run()
    chatbot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
