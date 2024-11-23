import typer
import inquirer
from .reactive_navigation import run_reactive_navigation
from .map_navigation import run_map_based_navigation

app = typer.Typer()

# Função para processar a escolha do usuário
def processar_respostas(respostas):

    if respostas["opcao"] == "Reativa":
        typer.echo("Iniciando navegação reativa...")
        run_reactive_navigation() 
    elif respostas["opcao"] == "Mapa":
        typer.echo("Iniciando navegação por mapa...")
        run_map_based_navigation()  

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