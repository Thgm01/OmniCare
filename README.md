# TCC

# Proposta
Fazer a proposta do projeto

# Integrantes
- Gustavo Rosell  	- Eng. Automação e Controle 	- FEI
- Júlia Hernandes   - Eng. Elétrica               - FEI
- Lucas Lagoeiro  	- Eng. de Robôs			          - FEI
- Massiel Blandy	  - Eng. Automação e Controle   - FEI
- Thiago Moura  	  - Eng. Automação e Controle 	- FEI


# Processo de Desenvolvimento
```mermaid

    flowchart TD
    %% Nodes
        subgraph man_sim ["Manipulação"]
            treed("Modelo 3D do robo Fusion 360")
            plug("Plugin de conversão")
            click plug href "https://youtu.be/_ZFo6wPXjeQ?si=RNDlK355hsLFP1IQ" "Open this in a new tab" _blank
            urdf_man("URDF")
            config_mov("Configuração do MoveIt2")
        end
        treed -->plug
        plug-->urdf_man
        urdf_man --> config_mov

        subgraph nav_sim["Navegação"]
            urdf_nav("urdf nav")
            check("check")
            pkg_nav_sim("pkg")
            plug_nav_sim("plug")
        end
        urdf_nav --> plug_nav_sim

        sim("Simulação")
        sim --> man_sim
        sim -->nav_sim



```