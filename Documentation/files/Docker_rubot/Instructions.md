# Instructions for LIMO in ROS1

Estos son los pasos para implementar el fix:
Para el docker si está corriendo: docker compose -f /path/to/agilex_limo/limo_ros/docker/docker-compose.yaml down (si mal no recuerdo, el path en donde estaba era ~/agilex_limo/limo_ros/docker/docker-compose.yaml)
Prune de docker para borrar caché: docker system prune
git pull de nuestro repositorio agilex_limo: cd ~/agilex_limo/ && git pull
Corre el docker en background: docker compose -f ~/agilex_limo/limo_ros/docker/docker-compose.yaml up -d
Para arreglar el mismo problema pero en tu RUbot, aquí tienes las adiciones que he hecho en agilex_limo, que podrás traspasar a cualquier robot en noetic:

https://bitbucket.org/theconstructcore/agilex_limo/commits/6d180fac825410b5212ebfd329b3bc622fb8fcdd

Creo que el problema es que en tu robot LIMO, no has dado los permisos suficientes a docker. Es por esto que tienes que agregar sudo a cada comando de docker. Es muy importante en este fix que haya permisos de docker. Puedes hacerlo de esta forma: (https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) 
Create docker group: sudo groupadd docker
Add user to group: sudo usermod -aG docker $USER
Log out and log back in for groups to be evaluated.
Test docker ps without sudo. It should work.

Lo que hacer es lo siguiente (ya con los permisos de docker suficientes):
Si ya tienes un LIMO registrado, no lo borres. Simplemente copia el comando que compartí en el screenshot y pégalo dentro del LIMO.
Asegúrate de que no haya ningún docker container corriendo.
docker system prune en el LIMO
Corre el docker-compose.yaml actualizado: https://bitbucket.org/theconstructcore/agilex_limo/src/master/limo_ros/docker/docker-compose.yaml
Sin hacer nada más dentro del robot, connecta al LIMO dentro de nuestra plataforma.
Haz test de movimiento de ruedas.
Desconecta/conecta desde nuestra web.
Haz test de movimiento de ruedas.