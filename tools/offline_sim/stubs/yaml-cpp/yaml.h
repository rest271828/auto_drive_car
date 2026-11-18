// 极简 YAML 读取: 只支持本项目配置文件用到的 "缩进 key: value" 结构和 # 注释
#pragma once
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace YAML
{
    struct Exception : public std::runtime_error
    {
        using std::runtime_error::runtime_error;
    };

    class Node
    {
    public:
        Node() : data_(std::make_shared<Data>()) {}

        Node operator[](const std::string &key) const
        {
            auto it = data_->children.find(key);
            if (it == data_->children.end())
            {
                throw Exception("missing key: " + key);
            }
            return it->second;
        }

        template <typename T>
        T as() const
        {
            std::stringstream ss(data_->value);
            T v{};
            ss >> v;
            if (ss.fail())
            {
                throw Exception("bad value: " + data_->value);
            }
            return v;
        }

        Node &child(const std::string &key) { return data_->children[key]; }
        void set_value(const std::string &v) { data_->value = v; }

    private:
        struct Data
        {
            std::string value;
            std::map<std::string, Node> children;
        };
        std::shared_ptr<Data> data_;
    };

    template <>
    inline std::string Node::as<std::string>() const
    {
        std::string v = data_->value;
        if (v.size() >= 2 && v.front() == '"' && v.back() == '"')
        {
            v = v.substr(1, v.size() - 2);
        }
        return v;
    }

    inline std::string trim(const std::string &s)
    {
        const auto b = s.find_first_not_of(" \t\r");
        if (b == std::string::npos)
        {
            return "";
        }
        const auto e = s.find_last_not_of(" \t\r");
        return s.substr(b, e - b + 1);
    }

    inline Node LoadFile(const std::string &path)
    {
        std::ifstream in(path);
        if (!in)
        {
            throw Exception("cannot open " + path);
        }
        Node root;
        std::vector<std::pair<int, Node *>> stack{{-1, &root}};
        std::string line;
        while (std::getline(in, line))
        {
            const auto hash = line.find('#');
            if (hash != std::string::npos)
            {
                line = line.substr(0, hash);
            }
            if (trim(line).empty())
            {
                continue;
            }
            const int indent = line.find_first_not_of(' ');
            const auto colon = line.find(':');
            const std::string key = trim(line.substr(0, colon));
            const std::string value = trim(line.substr(colon + 1));
            while (stack.back().first >= indent)
            {
                stack.pop_back();
            }
            Node &node = stack.back().second->child(key);
            node.set_value(value);
            stack.push_back({indent, &node});
        }
        return root;
    }
}
