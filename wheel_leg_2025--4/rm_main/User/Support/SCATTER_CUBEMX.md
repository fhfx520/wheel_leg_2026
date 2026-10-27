# CubeMX 与自定义 SCT

当前采用编译前同步，不再依赖 CubeMX 生成后钩子修复工程引用。
CubeMX 生成后，Linker 页显示 rm_main.sct 是正常状态。

1. 唯一维护的源文件是 User/Log/fault_noinit.sct。
2. Keil 保持未勾选 Use Memory Layout from Target Dialog，Scatter File 为 rm_main.sct。
3. 现有 Before Build 命令执行 User/Support/sync_dma_memory.ps1。
4. 脚本读取 ioc 更新 DMA 配置，再执行 sync_keil_scatter.ps1，
   将自定义布局同步为 MDK-ARM/rm_main.sct，然后编译链接。

源 SCT 不被修改。生成 SCT 的头文件路径按当前工程位置计算，
以解决复制后包含文件的相对位置变化。不要直接维护生成的 rm_main.sct。
CubeMX 刚生成后可能仍看到默认布局，编译前会自动刷新。

构建输出应出现 Scatter synchronized。Before Build 已设置为退出码大于等于
1 时停止。另有链接符号保护：缺少 RW_LOG_SPI_DMA 时，将报
Undefined symbol Image$$RW_LOG_SPI_DMA$$Base，避免静默生成错误布局固件。

旧 restore_keil_scatter.ps1 属于此前的生成后修改引用方案，现在不使用。
post_generate.bat 仍可手动运行，但现在只同步 DMA 配置和 SCT，不修改工程引用。
CubeMX 生成后脚本配置已清空；旧界面即使仍调用该批处理也只会同步内容。

本次变更后重新加载 Keil 工程，以读取停止构建设置。
工程迁移后需更新 Keil Before Build 中脚本及 ioc 的路径。
