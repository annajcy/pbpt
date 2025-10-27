
"""
构建 conan_recipe 目录下的所有自定义 Conan recipes

这个脚本会遍历 conan_recipe 目录，为每个包含 conanfile.py 的子目录
执行 conan create 命令，自动将本地 recipes 导出到本地 Conan 缓存。

用法:
    python script/build_conan_recipes.py                    # 使用默认 build_type
    python script/build_conan_recipes.py --build-type Release
    python script/build_conan_recipes.py -d conan_recipe    # 指定 recipe 目录
"""

import argparse
import subprocess
import sys
import io
from pathlib import Path
from typing import List


# Fix encoding issues on Windows
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')


class ConanRecipeBuilder:
    """Conan Recipe 构建器"""
    
    def __init__(self, recipe_dir: Path, build_type: str = "Debug", 
                 verbose: bool = False, dry_run: bool = False):
        """
        初始化构建器
        
        Args:
            recipe_dir: conan_recipe 目录路径
            build_type: 构建类型 (Debug/Release)
            verbose: 是否显示详细输出
            dry_run: 是否只显示将要执行的命令而不实际执行
        """
        self.recipe_dir = recipe_dir
        self.build_type = build_type
        self.verbose = verbose
        self.dry_run = dry_run
        
    def find_recipes(self) -> List[Path]:
        """
        查找所有包含 conanfile.py 的子目录
        
        Returns:
            包含 conanfile.py 的目录路径列表
        """
        if not self.recipe_dir.exists():
            print(f"错误: 目录不存在: {self.recipe_dir}", file=sys.stderr)
            return []
        
        if not self.recipe_dir.is_dir():
            print(f"错误: 不是一个目录: {self.recipe_dir}", file=sys.stderr)
            return []
        
        recipes = []
        for item in sorted(self.recipe_dir.iterdir()):
            if not item.is_dir():
                continue
            
            conanfile = item / "conanfile.py"
            if conanfile.exists():
                recipes.append(item)
        
        return recipes
    
    def build_recipe(self, recipe_path: Path) -> bool:
        """
        构建单个 recipe
        
        Args:
            recipe_path: recipe 目录路径
            
        Returns:
            是否构建成功
        """
        recipe_name = recipe_path.name
        
        # 构建命令
        cmd = [
            "conan", "create", str(recipe_path),
            "-s", f"build_type={self.build_type}",
            "--build=missing"
        ]
        
        print(f"\n{'=' * 80}")
        print(f"📦 构建 Recipe: {recipe_name} ({self.build_type})")
        print(f"{'=' * 80}")
        
        if self.verbose or self.dry_run:
            print(f"命令: {' '.join(cmd)}")
        
        if self.dry_run:
            print("(dry-run 模式，跳过实际执行)")
            return True
        
        try:
            # 执行命令
            result = subprocess.run(
                cmd,
                cwd=recipe_path.parent,
                check=False,
                capture_output=not self.verbose,
                text=True
            )
            
            if result.returncode == 0:
                print(f" 成功构建: {recipe_name}")
                return True
            else:
                print(f" 构建失败: {recipe_name} (退出码: {result.returncode})", 
                      file=sys.stderr)
                if not self.verbose and result.stderr:
                    print("错误输出:", file=sys.stderr)
                    print(result.stderr, file=sys.stderr)
                return False
                
        except subprocess.CalledProcessError as e:
            print(f" 构建失败: {recipe_name}", file=sys.stderr)
            print(f"错误: {e}", file=sys.stderr)
            return False
        except Exception as e:
            print(f" 未知错误: {recipe_name}", file=sys.stderr)
            print(f"错误: {e}", file=sys.stderr)
            return False
    
    def build_all(self) -> int:
        """
        构建所有找到的 recipes
        
        Returns:
            失败的构建数量
        """
        recipes = self.find_recipes()
        
        if not recipes:
            print(f"  没有在 {self.recipe_dir} 中找到任何 recipes")
            return 0
        
        print(f"找到 {len(recipes)} 个 recipe(s):")
        for recipe in recipes:
            print(f"  - {recipe.name}")
        
        failures = 0
        for recipe in recipes:
            if not self.build_recipe(recipe):
                failures += 1
        
        # 打印摘要
        print(f"\n{'=' * 80}")
        print("构建摘要:")
        print(f"  总数: {len(recipes)}")
        print(f"  成功: {len(recipes) - failures}")
        print(f"  失败: {failures}")
        print(f"{'=' * 80}")
        
        return failures


def main():
    parser = argparse.ArgumentParser(
        description='构建 conan_recipe 目录下的所有自定义 Conan recipes',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                                 # 使用默认设置构建所有 recipes
  %(prog)s --build-type Release            # 使用 Release 模式
  %(prog)s -d custom_recipes               # 指定自定义 recipe 目录
  %(prog)s -v                              # 显示详细输出
  %(prog)s --dry-run                       # 只显示将要执行的命令
  
注意:
  - 这个脚本等同于 CI 中的 "Conan Build Custom Recipes" 步骤
  - 会自动将本地 recipes 导出到 Conan 本地缓存
  - 使用 --build=missing 参数，远程找不到时会使用本地 recipe
        """
    )
    
    parser.add_argument(
        '-d', '--recipe-dir',
        type=str,
        default='.',
        help='conan_recipe 目录路径'
    )
    
    parser.add_argument(
        '-b', '--build-type',
        type=str,
        default='Debug',
        choices=['Debug', 'Release', 'RelWithDebInfo', 'MinSizeRel'],
        help='构建类型 (默认: Debug)'
    )
    
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='显示详细输出'
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='只显示将要执行的命令，不实际执行'
    )
    
    parser.add_argument(
        '--fail-fast',
        action='store_true',
        help='遇到错误时立即停止（默认会尝试构建所有 recipes）'
    )
    
    args = parser.parse_args()
    recipe_dir = Path(args.recipe_dir).resolve()

    print("Conan Recipe 构建工具")
    print(f"Recipe 目录: {recipe_dir}")
    print(f"构建类型: {args.build_type}")
    print()
    
    # 创建构建器
    builder = ConanRecipeBuilder(
        recipe_dir=recipe_dir,
        build_type=args.build_type,
        verbose=args.verbose,
        dry_run=args.dry_run
    )
    
    # 构建所有 recipes
    try:
        failures = builder.build_all()
        
        if failures > 0:
            print(f"\n  {failures} 个 recipe(s) 构建失败", file=sys.stderr)
            return 1
        else:
            print("\n 所有 recipes 构建成功!")
            return 0
            
    except KeyboardInterrupt:
        print("\n  用户中断", file=sys.stderr)
        return 130
    except Exception as e:
        print(f"\n 未预期的错误: {e}", file=sys.stderr)
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
