# `CPU`虚拟化

内核版本`linux-5.17.5`

`kvmtool`: http://git.kernel.org/pub/scm/linux/kernel/git/will/kvmtool.git

对于虚拟化的学习，本系列以`linux-5.17.5`以及`kvmtool`为工具。

## `KVM`初始化

初始化`KVM`时，需要加载`kvm.ko`以及`kvm-intel.ko`两个内核模块。其中`kvm.ko`由`KVM`的通用代码生成，`kvm-intel.ko`是由`Intel CPU`架构相关代码生成。最终`kvmtool`和`kvm`的交互由`kvm.ko`导出的设备`/dev/kvm`完成。

整个`KVM`的初始化由函数`vmx_init`来完成，其主要调用了`kvm_init`函数。

```c
static int __init vmx_init(void)
{
    int r, cpu;
    r = kvm_init(&vmx_init_ops, sizeof(struct vcpu_vmx),
             __alignof__(struct vcpu_vmx), THIS_MODULE);
    ...
    return 0;
}
```

函数`kvm_init`中的`vmx_init_ops`参数表示`Intel VT-x`实现的各种回调函数:

```c
static struct kvm_x86_init_ops vmx_init_ops __initdata = {
    .cpu_has_kvm_support = cpu_has_kvm_support,
    .disabled_by_bios = vmx_disabled_by_bios,
    .check_processor_compatibility = vmx_check_processor_compat,
    .hardware_setup = hardware_setup,
    .handle_intel_pt_intr = NULL,

    .runtime_ops = &vmx_x86_ops,
};
```

`vmx_init_ops`中内嵌变量`runtime_ops`由`vmx_x86_ops`指定。可以看到`vmx_x86_ops`才是真正的`Intel VT-x`回调函数集合：

```c
static struct kvm_x86_ops vmx_x86_ops __initdata = {
    .name = "kvm_intel",

    .hardware_unsetup = hardware_unsetup,

    .hardware_enable = hardware_enable,
    .hardware_disable = hardware_disable,
    .cpu_has_accelerated_tpr = report_flexpriority,
    .has_emulated_msr = vmx_has_emulated_msr,

    .vm_size = sizeof(struct kvm_vmx),
    .vm_init = vmx_vm_init,

    .vcpu_create = vmx_create_vcpu,
    .vcpu_free = vmx_free_vcpu,
    .vcpu_reset = vmx_vcpu_reset,

    .prepare_guest_switch = vmx_prepare_switch_to_guest,
    .vcpu_load = vmx_vcpu_load,
    .vcpu_put = vmx_vcpu_put,

    .update_exception_bitmap = vmx_update_exception_bitmap,
    .get_msr_feature = vmx_get_msr_feature,
    .get_msr = vmx_get_msr,
    .set_msr = vmx_set_msr,
    .get_segment_base = vmx_get_segment_base,
    .get_segment = vmx_get_segment,
    .set_segment = vmx_set_segment,
    .get_cpl = vmx_get_cpl,
    .get_cs_db_l_bits = vmx_get_cs_db_l_bits,
    .set_cr0 = vmx_set_cr0,
    .is_valid_cr4 = vmx_is_valid_cr4,
    .set_cr4 = vmx_set_cr4,
    .set_efer = vmx_set_efer,
    .get_idt = vmx_get_idt,
    .set_idt = vmx_set_idt,
    .get_gdt = vmx_get_gdt,
    .set_gdt = vmx_set_gdt,
    .set_dr7 = vmx_set_dr7,
    .sync_dirty_debug_regs = vmx_sync_dirty_debug_regs,
    .cache_reg = vmx_cache_reg,
    .get_rflags = vmx_get_rflags,
    .set_rflags = vmx_set_rflags,
    .get_if_flag = vmx_get_if_flag,

    .tlb_flush_all = vmx_flush_tlb_all,
    .tlb_flush_current = vmx_flush_tlb_current,
    .tlb_flush_gva = vmx_flush_tlb_gva,
    .tlb_flush_guest = vmx_flush_tlb_guest,

    .vcpu_pre_run = vmx_vcpu_pre_run,
    .run = vmx_vcpu_run,
    .handle_exit = vmx_handle_exit,
    .skip_emulated_instruction = vmx_skip_emulated_instruction,
    .update_emulated_instruction = vmx_update_emulated_instruction,
    .set_interrupt_shadow = vmx_set_interrupt_shadow,
    .get_interrupt_shadow = vmx_get_interrupt_shadow,
    .patch_hypercall = vmx_patch_hypercall,
    .set_irq = vmx_inject_irq,
    .set_nmi = vmx_inject_nmi,
    .queue_exception = vmx_queue_exception,
    .cancel_injection = vmx_cancel_injection,
    .interrupt_allowed = vmx_interrupt_allowed,
    .nmi_allowed = vmx_nmi_allowed,
    .get_nmi_mask = vmx_get_nmi_mask,
    .set_nmi_mask = vmx_set_nmi_mask,
    .enable_nmi_window = vmx_enable_nmi_window,
    .enable_irq_window = vmx_enable_irq_window,
    .update_cr8_intercept = vmx_update_cr8_intercept,
    .set_virtual_apic_mode = vmx_set_virtual_apic_mode,
    .set_apic_access_page_addr = vmx_set_apic_access_page_addr,
    .refresh_apicv_exec_ctrl = vmx_refresh_apicv_exec_ctrl,
    .load_eoi_exitmap = vmx_load_eoi_exitmap,
    .apicv_post_state_restore = vmx_apicv_post_state_restore,
    .check_apicv_inhibit_reasons = vmx_check_apicv_inhibit_reasons,
    .hwapic_irr_update = vmx_hwapic_irr_update,
    .hwapic_isr_update = vmx_hwapic_isr_update,
    .guest_apic_has_interrupt = vmx_guest_apic_has_interrupt,
    .sync_pir_to_irr = vmx_sync_pir_to_irr,
    .deliver_interrupt = vmx_deliver_interrupt,
    .dy_apicv_has_pending_interrupt = pi_has_pending_interrupt,

    .set_tss_addr = vmx_set_tss_addr,
    .set_identity_map_addr = vmx_set_identity_map_addr,
    .get_mt_mask = vmx_get_mt_mask,

    .get_exit_info = vmx_get_exit_info,

    .vcpu_after_set_cpuid = vmx_vcpu_after_set_cpuid,

    .has_wbinvd_exit = cpu_has_vmx_wbinvd_exit,

    .get_l2_tsc_offset = vmx_get_l2_tsc_offset,
    .get_l2_tsc_multiplier = vmx_get_l2_tsc_multiplier,
    .write_tsc_offset = vmx_write_tsc_offset,
    .write_tsc_multiplier = vmx_write_tsc_multiplier,

    .load_mmu_pgd = vmx_load_mmu_pgd,

    .check_intercept = vmx_check_intercept,
    .handle_exit_irqoff = vmx_handle_exit_irqoff,

    .request_immediate_exit = vmx_request_immediate_exit,

    .sched_in = vmx_sched_in,

    .cpu_dirty_log_size = PML_ENTITY_NUM,
    .update_cpu_dirty_logging = vmx_update_cpu_dirty_logging,

    .pmu_ops = &intel_pmu_ops,
    .nested_ops = &vmx_nested_ops,

    .update_pi_irte = pi_update_irte,
    .start_assignment = vmx_pi_start_assignment,

#ifdef CONFIG_X86_64
    .set_hv_timer = vmx_set_hv_timer,
    .cancel_hv_timer = vmx_cancel_hv_timer,
#endif

    .setup_mce = vmx_setup_mce,

    .smi_allowed = vmx_smi_allowed,
    .enter_smm = vmx_enter_smm,
    .leave_smm = vmx_leave_smm,
    .enable_smi_window = vmx_enable_smi_window,

    .can_emulate_instruction = vmx_can_emulate_instruction,
    .apic_init_signal_blocked = vmx_apic_init_signal_blocked,
    .migrate_timers = vmx_migrate_timers,

    .msr_filter_changed = vmx_msr_filter_changed,
    .complete_emulated_msr = kvm_complete_insn_gp,

    .vcpu_deliver_sipi_vector = kvm_vcpu_deliver_sipi_vector,
};
```

如下图所示以`kvm_init`为中心的函数调用图：

<img title="" src="./assets/kvm_init.png" alt="loading-ag-889" data-align="center">

函数说明

- `kvm_arch_init`:
  
  - `cpu_has_kvm_support`和`disabled_by_bios`:检测逻辑`cpu`是否支持`VMX`操作，以及是否被`Bios`关闭。
  
  - `kvm_mmu_vendor_module_init`:内存虚拟化初始化。
  
  - `kvm_timer_init`:时间虚拟化初始化。

- `kvm_arch_hardware_setup`:
  
  - `set_vmcs_config`根据当前逻辑`cpu`创建一个全局的`vmcs_config`用于与后续的每一个逻辑`cpu`的`vmcs_conf`做比较，若不一致则退出`KVM`初始化流程。
  - `cpu_has_vmx_ept`和`cpu_has_vmx_apicv`:根据`vmcs_config`和`MSR`值来设置某些全局变量，比如是否支持`EPT`的`enable_ept`变量()。
  - `kvm_configure_mmu`:根据全局变量`enable_ept`设置全局变量`tdp_enabled`以及`max_huge_page_level`。
  - `alloc_kvm_area()`:分配`vmxon`区域，并且放到`vmxarea`这个`percpu`变量中。

- `check_processor_compat`:检测所有的逻辑`cpu`的`vmcs`是否一致。

- `misc_register`:注册`misc`设备`/deev/kvm`。
  
  ```c
  r = misc_register(&kvm_dev);
  static struct miscdevice kvm_dev = {
      KVM_MINOR,
      "kvm",
      &kvm_chardev_ops,
  };
  static struct file_operations kvm_chardev_ops = {
      .unlocked_ioctl = kvm_dev_ioctl,
      .llseek        = noop_llseek,
      KVM_COMPAT(kvm_dev_ioctl),
  };
  ```

通过`misc_register(&kvm_dev)`就注册了`misc`设备`/dev/kvm`，可以看到`/dev/kvm`只支持`ioctl`系统调用。

`kvm_dev_ioctl`代码如下

```c
static long kvm_dev_ioctl(struct file *filp,
              unsigned int ioctl, unsigned long arg)
{
    long r = -EINVAL;

    switch (ioctl) {
    case KVM_GET_API_VERSION:
        if (arg)
            goto out;
        r = KVM_API_VERSION;
        break;
    case KVM_CREATE_VM:
        r = kvm_dev_ioctl_create_vm(arg);
        break;
    case KVM_CHECK_EXTENSION:
        r = kvm_vm_ioctl_check_extension_generic(NULL, arg);
        break;
    case KVM_GET_VCPU_MMAP_SIZE:
        if (arg)
            goto out;
        r = PAGE_SIZE;     /* struct kvm_run */
#ifdef CONFIG_X86
        r += PAGE_SIZE;    /* pio data page */
#endif
#ifdef CONFIG_KVM_MMIO
        r += PAGE_SIZE;    /* coalesced mmio ring page */
#endif
        break;
    case KVM_TRACE_ENABLE:
    case KVM_TRACE_PAUSE:
    case KVM_TRACE_DISABLE:
        r = -EOPNOTSUPP;
        break;
    default:
        return kvm_arch_dev_ioctl(filp, ioctl, arg);
    }
out:
    return r;
}
```

从上述代码可以看到`/dev/kvm`的`ioctl`接口分为两类：

- 通用接口:比如`KVM_API_VERSION`和`KVM_CREATE_VM`。

- 架构相关接口:`kvm_arch_dev_ioctl`。

此外，参见如下图，整个`KVM`还包括`VM`层面的`ioctl`以及`VCPU`层面的`ioctl`。

<img title="" src="./assets/kvm_ioctl.png" alt="kvm_ioctl.png" data-align="center">

`KVM`的初始化流程没有使逻辑`cpu`进入`VMX`模式，因为在整个`vmx_init`流程中并没有将`CR4.VMXE[bit 13]`设置为`1`。`VMX`模式的开启是在创建第一个虚拟机的时候。

## 虚拟机的创建

创建`KVM`虚拟机需要用户态的`kvmtool`发起。在`kvmtool`代码中，创建`KVM`虚拟机的流程由`kvm__init`发起。`kvmtool`和`kvm`的交互如下图所示:

<img title="" src="./assets/kvm_vm_create.png" alt="kvm_vm_create.png" data-align="center">

### `kvmtool`

- 打开`/dev/kvm`设备文件，获取此设备文件的`fd`。

- 根据`1.`中的`fd`，发起`KVM_CREATE_VM`调用，请求`kvm`创建虚拟机。代码如下：

```c
int kvm__init(struct kvm *kvm)
{
    int ret;

    // kvm__arch_cpu_supports_vm: 使用cpuid指令检测硬件环境是否支持虚拟化
    if (!kvm__arch_cpu_supports_vm()) {
        pr_err("Your CPU does not support hardware virtualization");
        ret = -ENOSYS;
        goto err;
    }

    // open /dev/kvm
    kvm->sys_fd = open(kvm->cfg.dev, O_RDWR);
    ...

    // KVM_GET_API_VERSION: Get the API version as the stable kvm API
    ret = ioctl(kvm->sys_fd, KVM_GET_API_VERSION, 0);
    ...

    // KVM_CREATE_VM: 创建虚拟机
    kvm->vm_fd = ioctl(kvm->sys_fd, KVM_CREATE_VM, kvm__get_vm_type(kvm));
    ...

    // query about extensions to the core kvm API.
    if (kvm__check_extensions(kvm)) {
        pr_err("A required KVM extension is not supported by OS");
        ret = -ENOSYS;
        goto err_vm_fd;
    }

    kvm__arch_init(kvm);
    ...
}
```

### `kvm`

在内核`KVM`模块中，函数`kvm_dev_ioctl`根据`kvmtool`发起`ioctl`调用时传入的`KVM_CREATE_VM`最终调用到`kvm_dev_ioctl_create_vm`函数，

- `kvm_create_vm`:创建虚拟机的核心函数。

- `kvm_coalesced_mmio_init``:对``coalesced MMIO`进行初始化。其实就是分配了一个`struct page`，然后将此`page`所在的虚拟地址赋值给`kvm->coalesced_mmio_ring`。

- `get_unused_fd_flags`:获取一个未被使用的`fd`，此`fd`作为`VM`层面调用`ioctl`时的`fd`。

- `anon_inode_getfile`:创建一个匿名的文件实例。

- `fd_install`:将`get_unused_fd_flags`获取的`fd`和`anon_inode_getfile`创建的匿名文件实例关联起来。

#### `kvm_create_vm`

作为创建虚拟机核心函数，`kvm_create_vm`调用关系如下:

<img title="" src="./assets/kvm_create_vm.png" alt="kvm_create_vm.png" data-align="center">

精简后的代码如下所示：

```c
static struct kvm *kvm_create_vm(unsigned long type)
{
    struct kvm *kvm = kvm_arch_alloc_vm();
    struct kvm_memslots *slots;
    int r = -ENOMEM;
    int i, j;
    ...
    mmgrab(current->mm);
    kvm->mm = current->mm;
    kvm_eventfd_init(kvm);
    ...
    xa_init(&kvm->vcpu_array);
    ...
    refcount_set(&kvm->users_count, 1);
    for (i = 0; i < KVM_ADDRESS_SPACE_NUM; i++) {
        for (j = 0; j < 2; j++) {
            slots = &kvm->__memslots[i][j];

            atomic_long_set(&slots->last_used_slot, (unsigned long)NULL);
            slots->hva_tree = RB_ROOT_CACHED;
            slots->gfn_tree = RB_ROOT;
            hash_init(slots->id_hash);
            slots->node_idx = j;

            /* Generations must be different for each address space. */
            slots->generation = i;
        }

        rcu_assign_pointer(kvm->memslots[i], &kvm->__memslots[i][0]);
    }

    for (i = 0; i < KVM_NR_BUSES; i++) {
        rcu_assign_pointer(kvm->buses[i],
            kzalloc(sizeof(struct kvm_io_bus), GFP_KERNEL_ACCOUNT));
        if (!kvm->buses[i])
            goto out_err_no_arch_destroy_vm;
    }

    kvm->max_halt_poll_ns = halt_poll_ns;

    r = kvm_arch_init_vm(kvm, type);
    ...

    r = hardware_enable_all();
    ...

#ifdef CONFIG_HAVE_KVM_IRQFD
    INIT_HLIST_HEAD(&kvm->irq_ack_notifier_list);
#endif

    r = kvm_init_mmu_notifier(kvm);
    ...
    r = kvm_arch_post_init_vm(kvm);
    ...
    list_add(&kvm->vm_list, &vm_list);

    preempt_notifier_inc();
    kvm_init_pm_notifier(kvm);

    /*
     * When the fd passed to this ioctl() is opened it pins the module,
     * but try_module_get() also prevents getting a reference if the module
     * is in MODULE_STATE_GOING (e.g. if someone ran "rmmod --wait").
     */
    if (!try_module_get(kvm_chardev_ops.owner)) {
        r = -ENODEV;
        goto out_err;
    }

    return kvm;

out_err:
    ...
    return ERR_PTR(r);
}
```

- `kvm_arch_alloc_vm`:分配一个`struct kvm`指针，用于表示一台虚拟机。

- `kvm_arch_init_vm`:初始化`struct kvm->arch`变量。

- `hardware_enable_all`:设置`CR4.VMXE[bit 13] = 1`使能`VMX`模式；对每一个逻辑`cpu`调用`vmxon`指令使其进入`VMX`模式。

- `list_add(&kvm->vm_list, &vm_list)`:将`kvm->vm_list`挂到以`vm_list`为头节点的链表上。

## `vCPU`的创建

虚拟机创建完成后，`kvmtool`可以根据其调用`KVM_CREATE_VM`时返回的虚拟机`fd`发起新的`ioctl`调用，请求创建`vCPU`。在`kvmtool`中，每一个`vCPU`对应宿主机操作系统中的一个用户态线程。在创建单个`vCPU`时，`kvmtool`和`kvm`的交互如下图。

<img title="vcpu_create" src="./assets/vcpu_create.png" alt="vcpu_create.png" data-align="center">

图中`kvmtool`侧`vcpu->kvm->vmfd`就是[虚拟机的创建](#虚拟机的创建)中由`KVM_CREATE_VM`返回的虚拟机`fd`。虚拟机`fd`所关联的匿名文件`file_operations`在`kvm`中定义如下：

```c
static struct file_operations kvm_vm_fops = {
    .release        = kvm_vm_release,
    .unlocked_ioctl = kvm_vm_ioctl,
    .llseek        = noop_llseek,
    KVM_COMPAT(kvm_vm_compat_ioctl),
};
```

`kvm`侧函数简介:

- `vcpu = kmem_cache_zalloc`:分配`struct kvm_vcpu`指针变量，此变量是一个`kvm_vcpu_cache`。

- `kvm_vcpu_init(vcpu, kvm, id)`:初始化由`kmem_cache_zalloc`分配的指针变量，将虚拟机所属的`struct kvm`变量和其关联(`vcpu->kvm = kvm` )。

- **`kvm_arch_vcpu_create(vcpu)`**:此函数是创建`vCPU`的核心函数之一，其最终调用了`vmx_x86_ops`中注册的`vmx_create_vcpu`函数。

- `kvm_get_kvm(kvm)`:每创建一个`vCPU`就将该虚拟机对应`kvm->users_count`加`1`。

- **`create_vcpu_fd(vcpu)`**:为新创建的`vcpu`关联一个匿名文件并且返回此匿名文件的文件描述符作为当前`vcpu`的`fd`，此`fd`返回给`kvmtool`使用；此`fd`关联的`file_operations`为`kvm_vcpu_fops`:
  
  ```c
  static struct file_operations kvm_vcpu_fops = {
      .release        = kvm_vcpu_release,
      .unlocked_ioctl = kvm_vcpu_ioctl,
      .mmap           = kvm_vcpu_mmap,
      .llseek        = noop_llseek,
      KVM_COMPAT(kvm_vcpu_compat_ioctl),
  };
  ```

- **`kvm_arch_vcpu_postcreate(vcpu)`**: 分别调用`vmx_vcpu_load`和`vmx_vcpu_put`。

### `kvm`

由于在创建`vcpu`时，`kvmtool`侧只需调用`KVM_CREATE_VCPU`即可。因此接下来重点分析`kvm`侧的代码。`kvm`侧代码参考如下图:![kvm_vm_ioctl_create_vcpu](./assets/kvm_vm_ioctl_create_vcpu.png)
