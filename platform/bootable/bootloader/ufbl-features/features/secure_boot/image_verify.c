/*
 * Copyright (C) 2015 - 2019 Amazon.com Inc. or its affiliates.  All Rights Reserved.
 */

#ifndef SUPPORT_BOLT
#include <string.h>
#ifndef UFBL_PLATFORM_AML
#include <debug.h>
#endif
#include <stddef.h>
#endif
#include "amzn_secure_boot.h"
#include "amzn_ufbl_alloc.h" /* for plat_alloc, plat_free */
#include "ufbl_debug.h" /* for dprintf */

#ifndef SHA256_DIGEST_LENGTH
#define SHA256_DIGEST_LENGTH (32)
#endif

extern int amzn_verify_image(int cert_type, char *computed_hash,
		const unsigned char *usercert, meta_data_handler handler);

__WEAK const char *amzn_target_device_name()
{
	return "invalid";
}

__WEAK int amzn_target_device_type(void)
{
	return AMZN_INVALID_DEVICE;
}

const unsigned char *amzn_get_kernel_cert(int cert_type, size_t *length)
{
	static const unsigned char production_cacert[] =
		"\x30\x82\x04\x2b\x30\x82\x02\xe3\xa0\x03\x02\x01\x02\x02\x09\x00"
		"\x84\xa7\xc2\xae\x52\xf0\xc4\xfb\x30\x3d\x06\x09\x2a\x86\x48\x86"
		"\xf7\x0d\x01\x01\x0a\x30\x30\xa0\x0d\x30\x0b\x06\x09\x60\x86\x48"
		"\x01\x65\x03\x04\x02\x01\xa1\x1a\x30\x18\x06\x09\x2a\x86\x48\x86"
		"\xf7\x0d\x01\x01\x08\x30\x0b\x06\x09\x60\x86\x48\x01\x65\x03\x04"
		"\x02\x01\xa2\x03\x02\x01\x20\x30\x7c\x31\x0b\x30\x09\x06\x03\x55"
		"\x04\x06\x13\x02\x55\x53\x31\x13\x30\x11\x06\x03\x55\x04\x08\x0c"
		"\x0a\x43\x61\x6c\x69\x66\x6f\x72\x6e\x69\x61\x31\x12\x30\x10\x06"
		"\x03\x55\x04\x07\x0c\x09\x53\x75\x6e\x6e\x79\x76\x61\x6c\x65\x31"
		"\x16\x30\x14\x06\x03\x55\x04\x0a\x0c\x0d\x41\x6d\x61\x7a\x6f\x6e"
		"\x20\x4c\x61\x62\x31\x32\x36\x31\x2c\x30\x2a\x06\x03\x55\x04\x0b"
		"\x0c\x23\x43\x6f\x6d\x6d\x6f\x6e\x20\x4b\x65\x72\x6e\x65\x6c\x20"
		"\x53\x69\x67\x6e\x69\x6e\x67\x20\x50\x72\x6f\x64\x75\x63\x74\x69"
		"\x6f\x6e\x20\x43\x41\x30\x1e\x17\x0d\x31\x35\x30\x34\x31\x36\x32"
		"\x33\x33\x30\x31\x30\x5a\x17\x0d\x32\x35\x30\x32\x32\x32\x32\x33"
		"\x33\x30\x31\x30\x5a\x30\x7c\x31\x0b\x30\x09\x06\x03\x55\x04\x06"
		"\x13\x02\x55\x53\x31\x13\x30\x11\x06\x03\x55\x04\x08\x0c\x0a\x43"
		"\x61\x6c\x69\x66\x6f\x72\x6e\x69\x61\x31\x12\x30\x10\x06\x03\x55"
		"\x04\x07\x0c\x09\x53\x75\x6e\x6e\x79\x76\x61\x6c\x65\x31\x16\x30"
		"\x14\x06\x03\x55\x04\x0a\x0c\x0d\x41\x6d\x61\x7a\x6f\x6e\x20\x4c"
		"\x61\x62\x31\x32\x36\x31\x2c\x30\x2a\x06\x03\x55\x04\x0b\x0c\x23"
		"\x43\x6f\x6d\x6d\x6f\x6e\x20\x4b\x65\x72\x6e\x65\x6c\x20\x53\x69"
		"\x67\x6e\x69\x6e\x67\x20\x50\x72\x6f\x64\x75\x63\x74\x69\x6f\x6e"
		"\x20\x43\x41\x30\x82\x01\x22\x30\x0d\x06\x09\x2a\x86\x48\x86\xf7"
		"\x0d\x01\x01\x01\x05\x00\x03\x82\x01\x0f\x00\x30\x82\x01\x0a\x02"
		"\x82\x01\x01\x00\x9d\xcd\xb4\xef\xa9\x18\x02\xc5\xea\x78\x55\x97"
		"\x4c\x11\x0e\xe4\x8f\xe0\xf5\xcf\x0b\x32\x3e\x8a\xf5\x40\x9e\x6d"
		"\xf7\xf9\xfa\xbc\x1b\x8f\x2e\xcf\x0a\x78\x00\x96\xdd\x30\x26\xe2"
		"\x0f\x50\xa2\x4c\x2c\x26\x23\xc5\x37\xef\x52\xc4\xcb\x0f\x91\xbf"
		"\x15\x9e\x09\xfa\xc1\x7f\x67\xd6\x45\x4c\x3d\x13\x34\x48\xcd\xbc"
		"\xfb\xae\x15\xe4\x14\x1f\x69\x44\x15\x19\x31\x62\xd6\x49\xe9\x0d"
		"\x10\x26\xe2\x11\xa9\x84\x15\xb9\xe5\x4a\xb4\x45\xc8\x92\x02\x6c"
		"\x80\xa4\xbb\xec\x94\xef\x04\x38\x7e\x92\x8e\x02\x1f\xf1\x84\x81"
		"\x05\xda\xf4\x48\xf6\xaa\xe7\xa8\xf3\x35\x95\xcd\x99\x85\x53\x8f"
		"\x81\xe7\x43\xa6\x2f\x4b\x24\x9f\xf1\x77\x4f\xf0\x16\xa2\x30\x3f"
		"\x54\xe1\x4d\x23\x99\xb7\x85\xf1\x28\x0a\xd8\xa4\xa5\xf3\xc0\xe4"
		"\x12\x2b\xb3\x69\x60\x28\x45\x81\x8a\x44\x6b\xf5\x68\xcf\x51\x93"
		"\x04\xa3\x4d\x35\x5c\x9f\x8a\x6f\x28\x69\xad\xcf\x7a\x23\x4b\x49"
		"\xf1\x4f\x25\x97\x50\xcd\x18\xa4\x5d\xa2\xd4\xfe\xfe\x9d\xf5\x62"
		"\x48\x95\x2e\x98\xa4\xa5\x0d\xb7\xf9\xf7\x0c\xdb\x37\x72\x97\xfa"
		"\x7b\x2b\xb7\xe5\x28\x43\xf1\x77\xac\x22\x19\x5b\x6a\xb5\x22\x9a"
		"\xa1\x2e\xe0\x67\x02\x03\x01\x00\x01\xa3\x50\x30\x4e\x30\x1d\x06"
		"\x03\x55\x1d\x0e\x04\x16\x04\x14\xc9\x8e\xce\x81\x40\x22\xc5\x98"
		"\x98\xb4\xcc\x31\x5c\xd6\x82\x0e\x67\xc3\xfa\x49\x30\x1f\x06\x03"
		"\x55\x1d\x23\x04\x18\x30\x16\x80\x14\xc9\x8e\xce\x81\x40\x22\xc5"
		"\x98\x98\xb4\xcc\x31\x5c\xd6\x82\x0e\x67\xc3\xfa\x49\x30\x0c\x06"
		"\x03\x55\x1d\x13\x04\x05\x30\x03\x01\x01\xff\x30\x3d\x06\x09\x2a"
		"\x86\x48\x86\xf7\x0d\x01\x01\x0a\x30\x30\xa0\x0d\x30\x0b\x06\x09"
		"\x60\x86\x48\x01\x65\x03\x04\x02\x01\xa1\x1a\x30\x18\x06\x09\x2a"
		"\x86\x48\x86\xf7\x0d\x01\x01\x08\x30\x0b\x06\x09\x60\x86\x48\x01"
		"\x65\x03\x04\x02\x01\xa2\x03\x02\x01\x20\x03\x82\x01\x01\x00\x07"
		"\x9f\x05\x00\x8e\xe1\xc4\x61\x17\x00\xea\xce\xeb\xb9\x18\x53\x6a"
		"\xcb\x95\x36\x23\x2b\xfc\x2e\x46\xa9\xc4\x30\xa0\x2f\x1c\x84\x24"
		"\xdd\x00\x52\x2b\x12\x49\xe4\xd4\xb0\xb5\x34\x20\x23\xc0\xac\x75"
		"\x29\x93\xae\xae\x8d\x5b\xc1\x0a\x1e\xa5\x54\x81\xf4\xc7\x47\x94"
		"\x82\x80\xed\x01\x61\x6b\xa9\x3b\x7f\x40\x26\x97\x05\xd8\x00\xf3"
		"\x1f\x09\xc7\x77\x79\xc5\x38\xf6\x71\xa8\x3b\x34\x9c\xe2\xd0\x5e"
		"\xab\x76\xff\x1f\xca\xb5\x81\xdc\x51\x7f\x19\x0e\x8b\xad\x31\x47"
		"\xbd\x45\xfd\xfb\xb0\x8d\xeb\xea\xc6\x47\xa3\x05\xb0\x29\x68\xd6"
		"\x96\x3a\x83\x09\xbf\xfb\xad\x64\x61\xbf\xde\xcf\xa5\x65\x4a\x35"
		"\x89\xb1\xa0\xb2\xbe\xa5\xf1\x3c\x98\x00\xc6\x9a\x65\xd7\x4a\x3e"
		"\x31\xf1\x01\x25\x3f\xb2\x1a\x2e\xa1\xf4\xdb\x35\x62\x52\x6d\xe1"
		"\xf3\xe0\x86\x80\xe4\x76\xc1\x30\x94\x03\x0f\x1c\xc1\xea\xae\x1d"
		"\x8c\xb3\x36\x7e\xbf\x46\x44\x4c\xd3\xeb\x36\xeb\x71\x90\xf6\xd0"
		"\x08\x9a\x93\x2f\x67\xdd\x97\xb2\x09\x5d\x1a\xeb\x71\xc5\x1d\xab"
		"\x2a\x7d\xb4\xfd\xfe\x32\x31\x0b\xe1\x00\xc4\xd1\x40\xb3\x70\x31"
		"\x62\xb5\xea\x24\x42\x36\x8a\x8d\x89\x26\xad\x48\xaf\x67\x50"
		;

	static const int production_cacert_size = 1071;

	static const unsigned char engineering_cacert[] =
		"\x30\x82\x04\x2d\x30\x82\x02\xe5\xa0\x03\x02\x01\x02\x02\x09\x00"
		"\xfd\x52\x91\x0b\x2f\xbe\x3c\x82\x30\x3d\x06\x09\x2a\x86\x48\x86"
		"\xf7\x0d\x01\x01\x0a\x30\x30\xa0\x0d\x30\x0b\x06\x09\x60\x86\x48"
		"\x01\x65\x03\x04\x02\x01\xa1\x1a\x30\x18\x06\x09\x2a\x86\x48\x86"
		"\xf7\x0d\x01\x01\x08\x30\x0b\x06\x09\x60\x86\x48\x01\x65\x03\x04"
		"\x02\x01\xa2\x03\x02\x01\x20\x30\x7d\x31\x0b\x30\x09\x06\x03\x55"
		"\x04\x06\x13\x02\x55\x53\x31\x13\x30\x11\x06\x03\x55\x04\x08\x0c"
		"\x0a\x43\x61\x6c\x69\x66\x6f\x72\x6e\x69\x61\x31\x12\x30\x10\x06"
		"\x03\x55\x04\x07\x0c\x09\x53\x75\x6e\x6e\x79\x76\x61\x6c\x65\x31"
		"\x16\x30\x14\x06\x03\x55\x04\x0a\x0c\x0d\x41\x6d\x61\x7a\x6f\x6e"
		"\x20\x4c\x61\x62\x31\x32\x36\x31\x2d\x30\x2b\x06\x03\x55\x04\x0b"
		"\x0c\x24\x43\x6f\x6d\x6d\x6f\x6e\x20\x4b\x65\x72\x6e\x65\x6c\x20"
		"\x53\x69\x67\x6e\x69\x6e\x67\x20\x45\x6e\x67\x69\x6e\x65\x65\x72"
		"\x69\x6e\x67\x20\x43\x41\x30\x1e\x17\x0d\x31\x35\x30\x32\x30\x31"
		"\x31\x38\x33\x32\x31\x30\x5a\x17\x0d\x32\x34\x31\x32\x31\x30\x31"
		"\x38\x33\x32\x31\x30\x5a\x30\x7d\x31\x0b\x30\x09\x06\x03\x55\x04"
		"\x06\x13\x02\x55\x53\x31\x13\x30\x11\x06\x03\x55\x04\x08\x0c\x0a"
		"\x43\x61\x6c\x69\x66\x6f\x72\x6e\x69\x61\x31\x12\x30\x10\x06\x03"
		"\x55\x04\x07\x0c\x09\x53\x75\x6e\x6e\x79\x76\x61\x6c\x65\x31\x16"
		"\x30\x14\x06\x03\x55\x04\x0a\x0c\x0d\x41\x6d\x61\x7a\x6f\x6e\x20"
		"\x4c\x61\x62\x31\x32\x36\x31\x2d\x30\x2b\x06\x03\x55\x04\x0b\x0c"
		"\x24\x43\x6f\x6d\x6d\x6f\x6e\x20\x4b\x65\x72\x6e\x65\x6c\x20\x53"
		"\x69\x67\x6e\x69\x6e\x67\x20\x45\x6e\x67\x69\x6e\x65\x65\x72\x69"
		"\x6e\x67\x20\x43\x41\x30\x82\x01\x22\x30\x0d\x06\x09\x2a\x86\x48"
		"\x86\xf7\x0d\x01\x01\x01\x05\x00\x03\x82\x01\x0f\x00\x30\x82\x01"
		"\x0a\x02\x82\x01\x01\x00\xb3\x23\xcc\x7d\x31\x56\xfc\x5c\xc6\xff"
		"\xfa\xde\xc1\xd2\xf1\x02\xb6\x63\x62\x42\xfe\x60\x63\x73\xc9\xa3"
		"\x8d\xdc\x7e\x44\xd5\x3a\x68\xc9\x54\x98\xfb\x18\x23\xf6\x9a\xae"
		"\xc3\x8f\x52\xa7\x7a\xa8\x3a\xeb\x96\x97\x80\x02\x89\x56\xc7\xd1"
		"\xe3\xc2\x39\x00\x29\x75\xef\xce\x36\x8f\xce\x8f\xa2\x7f\xd6\x9d"
		"\x26\x92\xf0\xf5\xeb\x7d\x3b\x65\x29\x1e\xb1\x60\x12\x57\x72\xd8"
		"\x79\x32\x28\xc1\xb9\x9b\xc5\xb8\xdb\x56\xe3\x5e\xbe\xaa\xa0\xef"
		"\xb9\x79\x0c\x14\x7b\x5d\x71\x99\xdc\xb4\xc6\xec\x36\x80\xaa\x3e"
		"\x76\x61\xdc\x38\x62\x9f\x6f\x8d\x6a\x5d\x01\x09\xc5\xa9\x09\x48"
		"\x0d\x1d\x0b\x5d\x50\xb5\xa5\xe8\xda\x43\xd8\xa9\x14\xd9\x82\x5d"
		"\x62\xca\xea\x13\x60\x73\x75\x15\x33\x9c\xa9\x24\x26\x5e\xe6\x23"
		"\x34\x3b\x1a\xb8\x77\x7f\x02\xa8\xce\x6d\xe3\x41\x77\x9e\xd5\x2d"
		"\x61\x71\x6d\xc5\x2e\x56\x06\x9b\xd0\xe4\x1a\x68\xfa\xa8\x80\x49"
		"\x3c\x4d\x08\x22\x9a\x33\x00\x57\x61\x5d\xe2\x7f\x57\x3d\x18\x68"
		"\xdb\x0e\x57\xc2\x76\x22\x54\x66\x2d\x32\x6c\xde\xdb\x2b\x4d\xf7"
		"\x0d\x03\xb2\x29\xc3\x96\xcb\xe5\x69\xf9\x11\x17\x1c\x0b\xd1\xee"
		"\x66\x6f\x30\x38\xf4\xdb\x02\x03\x01\x00\x01\xa3\x50\x30\x4e\x30"
		"\x1d\x06\x03\x55\x1d\x0e\x04\x16\x04\x14\xda\xfa\x9e\x4c\x48\x45"
		"\x54\x72\xe0\xa2\x4a\x55\xf8\xa8\x17\x42\x3e\x05\x37\x8c\x30\x1f"
		"\x06\x03\x55\x1d\x23\x04\x18\x30\x16\x80\x14\xda\xfa\x9e\x4c\x48"
		"\x45\x54\x72\xe0\xa2\x4a\x55\xf8\xa8\x17\x42\x3e\x05\x37\x8c\x30"
		"\x0c\x06\x03\x55\x1d\x13\x04\x05\x30\x03\x01\x01\xff\x30\x3d\x06"
		"\x09\x2a\x86\x48\x86\xf7\x0d\x01\x01\x0a\x30\x30\xa0\x0d\x30\x0b"
		"\x06\x09\x60\x86\x48\x01\x65\x03\x04\x02\x01\xa1\x1a\x30\x18\x06"
		"\x09\x2a\x86\x48\x86\xf7\x0d\x01\x01\x08\x30\x0b\x06\x09\x60\x86"
		"\x48\x01\x65\x03\x04\x02\x01\xa2\x03\x02\x01\x20\x03\x82\x01\x01"
		"\x00\x8b\x20\x63\x48\x4d\xec\xa5\x02\x4b\xc5\xed\x8d\x4b\x99\x2d"
		"\xcd\xd6\xcf\x3d\x58\xe0\x4f\xdc\xaa\xb7\x51\x45\xff\x59\x23\x45"
		"\x57\x79\x06\x26\xc9\xa5\xc0\xca\xef\xbf\xf6\x2f\x22\xaa\xca\x1e"
		"\x8a\x25\xdd\xfa\x5a\xa6\x79\xa4\x3b\x93\xf3\x2a\xf6\x2c\x80\xd9"
		"\x76\x33\xd3\x63\xa9\xaa\x4e\x66\x6e\x68\x61\x04\xb5\x2f\x35\xdf"
		"\x75\x3b\x08\xb4\x48\x9e\x7f\x18\xf3\x2e\xdd\xa9\xb5\xfd\xd9\x12"
		"\x16\x46\x20\x79\xf9\xce\x10\x6a\x3e\xa3\x71\x4c\x79\x7f\x55\x2e"
		"\x2a\xe7\x82\xc3\xa3\x68\xb5\x49\xc1\x10\xe5\x20\x3d\xc0\x27\xff"
		"\x8a\x45\x34\x74\x34\x79\xc9\x26\xb3\x0a\xb7\x7e\x79\x21\x6f\xe6"
		"\xe6\xb1\x12\x42\xad\x60\x29\xcf\x6c\x9e\xfe\xda\x96\xef\x50\x8b"
		"\x20\x01\x53\xd7\x09\x56\x2c\xe1\x8b\xc9\x91\xd7\xf2\x4e\xa2\x49"
		"\xda\x90\x93\x88\x09\xb8\x1b\x0f\x79\x73\x36\x0b\x39\xb1\xb8\x1a"
		"\xa4\x93\x0b\x0a\xa0\x91\xba\x63\xc3\x6d\xcb\x76\x93\x82\x13\xc2"
		"\x14\xbe\x7c\x43\x1f\x3f\x48\xd3\x19\x6d\x3e\xad\x0c\x7d\x52\x3a"
		"\x77\x31\x53\x3e\x09\x95\x5d\xfb\x65\x58\x8d\x41\x2e\xa0\x43\x50"
		"\x5d\xf9\x4d\x09\xc0\x0f\xb6\x87\x82\xee\x8b\x60\x13\xbf\x01\x72"
		"\xf8"
		;
	static const int engineering_cacert_size = 1073;

	if (cert_type == AMZN_ENGINEERING_CERT) {
		*length = engineering_cacert_size;
		return engineering_cacert;
	} else {
		*length = production_cacert_size;
		return production_cacert;
	}
}

/*
 * Returns 1 when image is signed and authorized.
 * Returns 0 when image is unauthorized.
 * Expects a pointer to the start of image and pointer to start of sig
 */
int
amzn_image_verify(const void *image,
		  unsigned char *signature,
		  unsigned int image_size, meta_data_handler handler)
{
	int auth = 0;
	char *digest = NULL;

	if (!(digest = amzn_plat_alloc(SHA256_DIGEST_LENGTH))) {
		dprintf(CRITICAL, "ERROR: Unable to allocate image hash\n");
		goto cleanup;
	}

	memset(digest, 0, SHA256_DIGEST_LENGTH);

	/*
	 * Calculate hash of image for comparison
	 */
	amzn_target_sha256(image, image_size, digest);

	if (amzn_verify_image(AMZN_PRODUCTION_CERT, digest,
					signature, handler)) {
		if (amzn_target_device_type() == AMZN_PRODUCTION_DEVICE) {
			dprintf(ALWAYS,
				"Image FAILED AUTHENTICATION on PRODUCTION device\n");
			/* Failed verification */
			goto cleanup;
		} else {
		        dprintf(ALWAYS,
				"Authentication failed on engineering device with production certificate\n");
		}

		if (amzn_target_device_type() != AMZN_ENGINEERING_DEVICE) {
			dprintf(ALWAYS,
				"%s: Unknown device type!\n", UFBL_STR(__FUNCTION__));
			goto cleanup;
		}

		/* Engineering device */
		if (amzn_verify_image(AMZN_ENGINEERING_CERT, digest,
					signature, handler)) {
			dprintf(ALWAYS,
				"Image FAILED AUTHENTICATION on ENGINEERING device\n");
			goto cleanup;
		}
	} else {
		dprintf(ALWAYS,
			"Image AUTHENTICATED with PRODUCTION certificate\n");
	}

	auth = 1;

cleanup:
	if (digest)
		amzn_plat_free(digest);

	return auth;
}
