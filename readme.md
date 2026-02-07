因为本机环境中安装了 pyenv,而本项目又希望使用系统的原生 python 环境。所以需要设置一下环境

在项目中添加文件 `.python-version`，里面的内容只写 `system` 即可。（项目中我已经提前写好了。此时 `.python-version` 的优先级是最高的。

1. 执行编译

```
colcon build
```

2. 安装一下 setup

```
source install/setup.bash
```

再以后就可以使用 `colcon build` 正常编译不会报错了。
