import rclpy
import threading
from .main_cli import main as main_cli
from .Utils.Csv import load_from_csv
from .Game import Game
from .Editor import Editor

def game():
    rclpy.init()
    game = Game(load_from_csv("maps/default.csv"))
    
    # Executa main_cli em uma thread separada para evitar bloqueio
    cli_thread = threading.Thread(target=main_cli)
    cli_thread.start()

    game_thread = threading.Thread(target=rclpy.spin, args=(game,))
    game_thread.start()

    try:
        while rclpy.ok():
            game.run()
    except KeyboardInterrupt:
        pass
    finally:
        game.destroy_node()
        rclpy.shutdown()
        game_thread.join()
        cli_thread.join()  # Aguarda a finalização do CLI

def editor():
    editor = Editor(load_from_csv("maps/default.csv"))
    editor.run()