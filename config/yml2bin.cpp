#include <DBoW3/DBoW3.h>
#include <iostream>

int main() {
    // 加载现有的yml格式的词汇表
    DBoW3::Vocabulary voc("vocab_larger.yml.gz");  // yml格式的词汇表文件

    if (voc.empty()) {
        std::cerr << "Vocabulary file is empty!" << std::endl;
        return -1;
    }

    // 保存词汇表为二进制格式
    voc.save("vocab_larger.bin");  // 保存为二进制文件
    std::cout << "Vocabulary saved as binary format." << std::endl;

    return 0;
}
