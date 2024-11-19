import typer
import inquirer
import logging
from .navegacao_reativa import main as main_reativa
from .navegacao_mapa import main as main_mapa

logging.basicConfig(level=logging.INFO)

app = typer.Typer()

# Função para processar a escolha do usuário
def processar_respostas(respostas):

    if respostas["opcao"] == "Reativa":
        logging.info("Iniciando navegação reativa...")
        typer.echo("Iniciando navegação reativa...")
        main_reativa() 
    elif respostas["opcao"] == "Mapa":
        logging.info("Iniciando navegação por mapa...")
        typer.echo("Iniciando navegação por mapa...")
        main_mapa()  

# Comando para escolher a navegação
@app.command()
def navegacao():
    perguntas = [
        inquirer.List("opcao", message="Escolha a navegação", choices=["Reativa", "Mapa"]),
    ]
    respostas = inquirer.prompt(perguntas)
    
    if respostas:
        processar_respostas(respostas)

def main():
    app()