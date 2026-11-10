#pragma once

#include <openssl/evp.h>

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

namespace fusionforce {

class Sha256 {
public:
    Sha256()
    {
        if (!context_ || EVP_DigestInit_ex(context_.get(), EVP_sha256(), nullptr) != 1) {
            throw std::runtime_error("No se pudo iniciar SHA-256");
        }
    }

    void update(std::string_view bytes)
    {
        if (!context_ || EVP_DigestUpdate(context_.get(), bytes.data(), bytes.size()) != 1) {
            throw std::runtime_error("No se pudo actualizar SHA-256");
        }
    }

    std::string finish()
    {
        std::array<unsigned char, 32> hash{};
        unsigned int size = 0;
        if (!context_ || EVP_DigestFinal_ex(context_.get(), hash.data(), &size) != 1 ||
            size != hash.size()) {
            throw std::runtime_error("No se pudo finalizar SHA-256");
        }
        context_.reset();
        constexpr std::string_view hex = "0123456789abcdef";
        std::string result;
        result.reserve(hash.size() * 2);
        for (const auto byte : hash) {
            result.push_back(hex[byte >> 4]);
            result.push_back(hex[byte & 0x0f]);
        }
        return result;
    }

private:
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context_{
        EVP_MD_CTX_new(), EVP_MD_CTX_free};
};

inline std::string sha256(std::string_view bytes)
{
    Sha256 hash;
    hash.update(bytes);
    return hash.finish();
}

} // namespace fusionforce
