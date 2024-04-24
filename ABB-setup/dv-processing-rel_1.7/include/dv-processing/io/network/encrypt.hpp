#pragma once

#include <boost/asio/ssl.hpp>

#include <filesystem>
#include <optional>

namespace dv::io {

namespace asioSSL = boost::asio::ssl;

namespace encrypt {

/**
 * Create an encryption context.
 * @param method Encryption mode.
 * @param certificateChain Path to certificate chain.
 * @param privateKey Path to a private key.
 * @param CAFile Path to CAFile, if a std::nullopt is provided, peer verification is disabled. Can be an empty
 * path, in that case the context will use CA from default locations, the peers will be verified.
 * @return Encryption context.
 */
[[nodiscard]] inline asioSSL::context createEncryptionContext(asioSSL::context::method method,
	const std::filesystem::path &certificateChain, const std::filesystem::path &privateKey,
	const std::optional<std::filesystem::path> &CAFile = std::nullopt) {
	asioSSL::context context = asioSSL::context(method);

	context.use_certificate_chain_file(certificateChain.string());

	context.use_private_key_file(privateKey.string(), asioSSL::context::pem);

	context.set_options(asioSSL::context::default_workarounds | asioSSL::context::single_dh_use);

	// Default: no client verification enforced.
	context.set_verify_mode(asioSSL::context::verify_none);

	if (CAFile.has_value()) {
		if (CAFile->empty()) {
			context.set_default_verify_paths();
		}
		else {
			context.load_verify_file(CAFile->string());
		}

		context.set_verify_mode(asioSSL::context::verify_peer | asioSSL::context::verify_fail_if_no_peer_cert);
	}

	return context;
}

/**
 * Create an encryption server context with default configuration: TLSv1.2 encryption algorithm, provided certificate
 * chain, server private key and certificate authority CAFile which is used to verify client certificate.
 * @param certificateChain Server certificate chain.
 * @param privateKey Server private key.
 * @param CAFile CAFile for client verification.
 * @return SSL context that can be used for encrypted network connections.
 */
[[nodiscard]] inline asioSSL::context defaultEncryptionServer(const std::filesystem::path &certificateChain,
	const std::filesystem::path &privateKey, const std::filesystem::path &CAFile) {
	return createEncryptionContext(asioSSL::context::tlsv12_server, certificateChain, privateKey, CAFile);
}

/**
 * Create an encrypted client context with default configuration: TLSv1.2 encryption algorithm, provided
 * client certificate chain and client private key. Server is always considered trusted and server certificate is
 * not verified, the server will verify the client and can reject the connection during handshake if certificate
 * verification fails.
 * @param certificateChain Client certificate chain.
 * @param privateKey Client private key.
 * @return SSL context that can be used with encrypted network connections.
 */
[[nodiscard]] inline asioSSL::context defaultEncryptionClient(
	const std::filesystem::path &certificateChain, const std::filesystem::path &privateKey) {
	return createEncryptionContext(asioSSL::context::tlsv12_client, certificateChain, privateKey);
}

} // namespace encrypt

} // namespace dv::io
