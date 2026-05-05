#!/bin/bash

# Obtém o diretório do script
script_dir=$(dirname "$0")

# Arrays de origens
origens=("$script_dir/mavlink" "$script_dir/mavros/mavros_msgs" "$script_dir/mavros/msg" "$script_dir/mavros/mavros_extras" "$script_dir/mavros/plugins")

# Solicita ao usuário o caminho de destino relativo à pasta home
read -p "Digite o caminho para o ardupilot (a partir de sua home): " destino_relative1
read -p "Digite o caminho para o seu workspace (a partir de sua home): " destino_relative2


destino2="$HOME/$destino_relative2/src/mavlink/message_definitions/v1.0"
destino3="$HOME/$destino_relative2/src/mavros/mavros_msgs"
destino4="$HOME/$destino_relative2/src/mavros/mavros_msgs/msg"
destino5="$HOME/$destino_relative2/src/mavros/mavros_extras"
destino6="$HOME/$destino_relative2/src/mavros/mavros_extras/src/plugins"

# Array de destinos associados às origens
destinos=("$destino2" "$destino3" "$destino4" "$destino5" "$destino6")

# Loop para copiar arquivos de origens para os destinos associados
for ((i=0; i<${#origens[@]}; i++)); do
    origem="${origens[i]}"
    destino="${destinos[i]}"
    
    # Verifique se a origem e o destino existem
    if [ -d "$origem" ] && [ -d "$destino" ]; then
        # Copie os arquivos da origem para o destino
        cp -r "$origem"/* "$destino"
        echo "Arquivos copiados com sucesso de $origem para $destino"
    else
        echo "O diretório de origem $origem ou o diretório de destino $destino não existe."
    fi
done

