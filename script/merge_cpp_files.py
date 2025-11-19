import argparse
from pathlib import Path
from typing import List

def find_source_files(directory: Path, extensions: List[str]) -> List[Path]:
    """
    递归查找指定目录下所有匹配扩展名的文件
    
    Args:
        directory: 要搜索的目录
        extensions: 文件扩展名列表，如 ['.hpp', '.cpp']
    
    Returns:
        匹配的文件路径列表，按路径排序
    """
    files: List[Path] = []
    for ext in extensions:
        files.extend(directory.rglob(f'*{ext}'))
    return sorted(files)


def merge_files(files: List[Path], output_file: Path, base_dir: Path):
    """
    将多个源文件合并到一个输出文件中
    
    Args:
        files: 要合并的文件列表
        output_file: 输出文件路径
        base_dir: 基础目录，用于生成相对路径
    """
    with open(output_file, 'w', encoding='utf-8') as outf:
        outf.write("# 合并的源代码文件\n")
        outf.write(f"# 基础目录: {base_dir}\n")
        outf.write(f"# 文件总数: {len(files)}\n")
        outf.write(f"{'=' * 80}\n\n")
        
        for file_path in files:
            try:
                # 获取相对路径
                rel_path = file_path.relative_to(base_dir)
                
                # 写入文件分隔符和文件信息
                outf.write(f"\n{'=' * 80}\n")
                outf.write(f"# 文件: {rel_path}\n")
                outf.write(f"{'=' * 80}\n\n")
                
                # 读取并写入文件内容
                with open(file_path, 'r', encoding='utf-8') as inf:
                    content = inf.read()
                    outf.write(content)
                    
                    # 确保文件内容以换行结尾
                    if content and not content.endswith('\n'):
                        outf.write('\n')
                
                print(f"已添加: {rel_path}")
                
            except Exception as e:
                print(f"警告: 无法读取文件 {file_path}: {e}")
                continue
        
        outf.write(f"\n{'=' * 80}\n")
        outf.write("# 合并完成\n")
        outf.write(f"{'=' * 80}\n")


def main():
    parser = argparse.ArgumentParser(
        description='将指定文件夹中的所有 .hpp 和 .cpp 文件合并到一个 .txt 文件中',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s src                          # 合并 src 目录下的文件到 src/merged_sources.txt
  %(prog)s src -o output.txt            # 合并到指定的输出文件
  %(prog)s src -e .h .cpp .cc           # 自定义文件扩展名
  %(prog)s src --no-recursive           # 不递归子目录
        """
    )
    
    parser.add_argument(
        'directory',
        type=str,
        help='要扫描的目录路径'
    )
    
    parser.add_argument(
        '-o', '--output',
        type=str,
        default=None,
        help='输出文件路径（默认为 <directory>/merged_sources.txt）'
    )
    
    parser.add_argument(
        '-e', '--extensions',
        nargs='+',
        default=['.hpp', '.cpp'],
        help='要合并的文件扩展名（默认: .hpp .cpp）'
    )
    
    parser.add_argument(
        '--no-recursive',
        action='store_true',
        help='不递归搜索子目录'
    )
    
    args = parser.parse_args()
    
    # 处理目录路径
    directory = Path(args.directory).resolve()
    
    if not directory.exists():
        print(f"错误: 目录不存在: {directory}")
        return 1
    
    if not directory.is_dir():
        print(f"错误: 不是一个目录: {directory}")
        return 1
    
    # 处理输出文件路径
    if args.output:
        output_file = Path(args.output)
        if not output_file.is_absolute():
            output_file = directory / output_file
    else:
        output_file = directory / 'merged_sources.txt'
    
    output_file = output_file.resolve()
    
    # 确保扩展名以点开头
    extensions = [ext if ext.startswith('.') else f'.{ext}' for ext in args.extensions]
    
    print(f"搜索目录: {directory}")
    print(f"文件扩展名: {', '.join(extensions)}")
    print(f"递归搜索: {'否' if args.no_recursive else '是'}")
    print(f"输出文件: {output_file}")
    print()
    
    # 查找文件
    if args.no_recursive:
        files = [f for f in directory.iterdir() 
                if f.is_file() and f.suffix in extensions]
        files = sorted(files)
    else:
        files = find_source_files(directory, extensions)
    
    if not files:
        print(f"警告: 在 {directory} 中没有找到匹配的文件")
        return 1
    
    print(f"找到 {len(files)} 个文件\n")
    
    # 合并文件
    try:
        merge_files(files, output_file, directory)
        print(f"\n成功! 所有文件已合并到: {output_file}")
        print(f"输出文件大小: {output_file.stat().st_size / 1024:.2f} KB")
        return 0
    except Exception as e:
        print(f"错误: 合并文件时出错: {e}")
        return 1


if __name__ == '__main__':
    exit(main())
