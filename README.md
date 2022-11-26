# NeonFC - Neon Futebol Clube

### Software de estratégia

Software desenvolvido para LARC 2020 para o VSSS online.

### Instalação

#### Para usar o NeonFC você irá precisar ter instalado em sua máquina:

- Python >=3.7.x
- PiPy
- Google protobuf

Além de uma interface que irá enviar os dados do campo. Atualmente o [FIRASim](https://github.com/VSSSLeague/FIRASim) é suportado.

#### passo-a-passo da instalação:

1. Clonar o repositório
```bash
git clone https://github.com/project-neon/NeonFC.git
```

2. Instalar dependências usando pipy:

```bash
cd NeonFC
pip3 install -r requirements.txt
```

2.1. Antes de executar, é importante checar o arquivo de configuração para verificar se as portas e endereços de rede estão compativeis com o software de simulação.

No FiraSIM:

![Aba de Comunicação do FIRASim](readme/FiraSIM-comtab.png "Aba de Comunicação do FIRASim")

No ```config.json```:

```json
{
    "network" : {
        "multicast_ip": "224.0.0.1",
        "host_ip": "localhost",
        "blue_port": 30011,
        "yellow_port": 30012,
        "vision_port": 10020,
        "command_port": 20011
    },
    "match" : {
        "robots": 5
    }
}
```

3. Agora é só rodar ```Python main.py```

### Desenvolvimento

Esse repositório é desenvolvido pela equipe Project Neon, mas sinta-se livre para Forkar!

Manual de desenvolvimento ainda esta WIP mas as documentação irá ser construida [aqui](https://github.com/project-neon/NeonFC/wiki).
