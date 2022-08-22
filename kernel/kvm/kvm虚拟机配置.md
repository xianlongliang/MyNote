# 1. 配置虚拟机支持虚拟化

在`xml`中配置虚拟机支持虚拟化以及`numa`:

```xml
<cpu mode='host-model'>
    <model fallback='allow'/>
    <feature policy='require' name='vmx'>
    <topology sockets='2' core='8' threads='1'/>
    <numa>
        <cell id='0' cpus='0-7' memory='6291456' unit='KiB'/>
        <cell id='1' cpus='8-15' memory='6291456' unit='KiB'/>
    </numa>
</cpu>
```


