#include <iostream>
#include <torch/script.h>
#include <torch/torch.h>

int main(int argc, char const *argv[]) {
    Net network(50, 10);
    std::cout << network << std::endl;
    torch::Tensor x, output;
    x = torch::randn({2, 50}, torch::kCUDA);
    std::cout << "Input x is : " << std::endl;
    std::cout << x << std::endl;
    output = network->forward(x);
    std::cout << "Output is :" << std::endl;
    std::cout << output << std::endl;

    torch::jit::script::Module net = torch::jit::load("../models/net.pt");
    torch::Tensor x = torch::randn({1, 100});
    torch::Tensor x = torch::randn({3, 3}, torch::kCUDA);
    std::cout << x << std::endl;
    std::vector<torch::jit::IValue> input;
    input.push_back(x);

    for (int i = 0; i < 10; ++i) {
        auto out = net.forward(input);
        std::cout << out << std::endl;
        // std::cout << typeid(out).name() << std::endl;
    }

    return 0;
}