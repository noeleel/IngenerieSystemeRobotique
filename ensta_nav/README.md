# Nav package


### Mapping

Start mapping:
```bash
roslaunch ensta_nav nav.launch mapping:=true
```

Save map:
```bash
rosrun map_server map_saver -f {map.yaml}
```


### Localisation

Start navigation with AMCL node:
```bash
roslaunch ensta_nav nav.launch amcl:=true map:={map}
```
