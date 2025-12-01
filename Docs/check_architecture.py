"""
STM32_485项目架构检查工具
自动检测代码是否符合分层规范
"""

import os
from pathlib import Path

# 定义分层规则
LAYER_RULES = {
    'Core/App': {
        'role': '业务编排层',
        'allowed': ['调用函数', '流程控制', '状态机'],
        'forbidden': ['复杂算法', '数据结构实现', '协议解析'],
        'max_lines': 200,  # 单文件最大行数
    },
    'Drivers/Middlewares': {
        'role': '中间件层',
        'allowed': ['协议栈', '算法库', '可复用组件'],
        'forbidden': ['硬件直接操作', 'GPIO读写', 'HAL_*调用'],
        'max_lines': 500,
    },
    'Drivers/BSP': {
        'role': '板级驱动层',
        'allowed': ['硬件封装', 'GPIO操作', 'HAL_*调用'],
        'forbidden': ['业务逻辑', '状态机', '算法实现'],
        'max_lines': 300,
    },
    'Drivers/SYSTEM': {
        'role': '系统基础设施',
        'allowed': ['delay', 'usart', 'sys', 'fifo'],
        'forbidden': ['业务逻辑', '协议解析'],
        'max_lines': 500,
    }
}

# 关键词检测（简化版）
KEYWORDS = {
    'business_logic': ['if.*motor_running', 'state.*=', 'switch.*state'],
    'hardware_direct': ['HAL_GPIO_', 'HAL_UART_Transmit', '__HAL_'],
    'algorithm': ['for.*i.*<.*100', 'pid_compute', 'filter_'],
}

def check_file(file_path):
    """检查单个文件"""
    issues = []
    
    # 获取相对路径
    rel_path = str(file_path).replace('\\', '/')
    
    # 判断属于哪一层
    layer = None
    for layer_name in LAYER_RULES.keys():
        if layer_name in rel_path:
            layer = layer_name
            break
    
    if not layer:
        return []
    
    # 读取文件
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            line_count = len(lines)
            content = ''.join(lines)
    except:
        return []
    
    # 检查1: 文件行数
    max_lines = LAYER_RULES[layer]['max_lines']
    if line_count > max_lines:
        issues.append({
            'file': rel_path,
            'type': 'SIZE',
            'severity': 'WARNING',
            'message': f'文件过大 ({line_count} 行 > {max_lines} 行限制)'
        })
    
    # 检查2: App层不应有复杂实现
    if 'Core/App' in rel_path and '.c' in rel_path:
        if line_count > 200:
            issues.append({
                'file': rel_path,
                'type': 'ARCH',
                'severity': 'ERROR',
                'message': 'App层文件过大，可能包含复杂实现逻辑'
            })
        
        # 检测算法关键词
        if any(kw in content for kw in ['for (int i', 'pid_compute', 'filter']):
            issues.append({
                'file': rel_path,
                'type': 'ARCH',
                'severity': 'ERROR',
                'message': 'App层不应包含算法实现'
            })
    
    # 检查3: Middlewares层不应直接操作硬件
    if 'Drivers/Middlewares' in rel_path and '.c' in rel_path:
        hw_calls = [kw for kw in ['HAL_GPIO_Write', 'HAL_UART_Transmit'] if kw in content]
        if hw_calls:
            issues.append({
                'file': rel_path,
                'type': 'ARCH',
                'severity': 'WARNING',
                'message': f'Middlewares层直接调用硬件: {hw_calls}'
            })
    
    # 检查4: BSP层不应有业务逻辑
    if 'Drivers/BSP' in rel_path and '.c' in rel_path:
        if 'switch (state)' in content or 'motor_running' in content:
            issues.append({
                'file': rel_path,
                'type': 'ARCH',
                'severity': 'ERROR',
                'message': 'BSP层不应包含业务逻辑或状态机'
            })
    
    return issues

def scan_project(root_dir):
    """扫描整个项目"""
    print("=" * 60)
    print("STM32_485 架构检查工具 V1.0")
    print("=" * 60)
    print()
    
    all_issues = []
    
    # 扫描所有.c/.h文件
    for ext in ['*.c', '*.h']:
        for file_path in Path(root_dir).rglob(ext):
            # 排除HAL库和第三方代码
            if any(x in str(file_path) for x in ['STM32F1xx_HAL_Driver', 'CMSIS', 'build']):
                continue
            
            issues = check_file(file_path)
            all_issues.extend(issues)
    
    # 分类统计
    errors = [i for i in all_issues if i['severity'] == 'ERROR']
    warnings = [i for i in all_issues if i['severity'] == 'WARNING']
    
    # 打印结果
    if errors:
        print("🚨 严重问题 (ERROR):")
        for issue in errors:
            print(f"  [{issue['type']}] {issue['file']}")
            print(f"    {issue['message']}")
            print()
    
    if warnings:
        print("⚠️  警告 (WARNING):")
        for issue in warnings:
            print(f"  [{issue['type']}] {issue['file']}")
            print(f"    {issue['message']}")
            print()
    
    if not all_issues:
        print("✅ 恭喜！未发现架构问题！")
        print()
    
    # 统计
    print("=" * 60)
    print(f"检查完成: {len(errors)} 个错误, {len(warnings)} 个警告")
    print("=" * 60)
    
    # 架构建议
    if errors or warnings:
        print()
        print("📋 整改建议:")
        print("  1. App层文件超过200行 → 提取功能到Middlewares层")
        print("  2. Middlewares层调用HAL_* → 通过BSP层封装")
        print("  3. BSP层有状态机 → 移动业务逻辑到App层")
        print("  4. 参考文档: Docs/ARCHITECTURE_GUIDE.md")
    
    return len(errors), len(warnings)

if __name__ == '__main__':
    import sys
    
    # 获取项目根目录
    root = Path(__file__).parent.parent
    
    errors, warnings = scan_project(root)
    
    # 返回退出码（CI/CD可用）
    sys.exit(1 if errors > 0 else 0)
