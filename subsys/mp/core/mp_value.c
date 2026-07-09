/*
 * Copyright 2025-2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/mp/core/mp_value.h>

LOG_MODULE_REGISTER(mp_value, CONFIG_MP_LOG_LEVEL);

#define MP_VALUE_SIMPLE(value)               ((struct mp_value_simple *)value)
#define MP_VALUE_RANGE(value)                ((struct mp_value_range *)value)
#define MP_VALUE_LIST(value)                 ((struct mp_value_list *)value)
#define MP_VALUE_CONST(value)                ((const mp_value_t)value)
#define MP_VALUE_SIMPLE_CONST(value)         ((const struct mp_value_simple *)value)
#define MP_VALUE_RANGE_CONST(value)          ((const struct mp_value_range *)value)
#define MP_VALUE_LIST_CONST(value)           ((const struct mp_value_list *)value)

/*
 * This is the value representation in binary for 32-bit systems, taking advantage
 * that all pointers involved are aligned on 32-bit:
 *
 * - xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx00 - a pointer to a @c VALUE, 'x' is the pointer
 * - xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx01 - a pointer to an @c OBJECT, 'x' is the pointer
 * - xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx10 - other pointer type (@c PTR), 'x' is the pointer
 * - xxxxxxxxxxxxxxxxxxxxxxxxxxxxtt11 - an immediate value, 't' is type, 'x' is the value
 */

#define MP_VALUE_IS_VALUE_PTR(value)	(((uintptr_t)(value) & 0x3U) == 0x0)
#define MP_VALUE_IS_OBJECT_PTR(value)	(((uintptr_t)(value) & 0x3U) == 0x1)
#define MP_VALUE_IS_OTHER_PTR(value)	(((uintptr_t)(value) & 0x3U) == 0x2)
#define MP_VALUE_IS_IMMEDIATE(value)	(((uintptr_t)(value) & 0x3U) == 0x3)

#define MP_VALUE_IS_FITTING(i) (((uint64_t)i >> 28) == 0)
#define MP_VALUE_IS_NULL(value)                                                                    \
	(!MP_VALUE_IS_IMMEDIATE(value) && MP_VALUE_GET_PTR(value) == NULL)
#define MP_VALUE_IS_VALID(value)                                                                   \
	IN_RANGE(mp_value_get_type(value), MP_TYPE_NONE, MP_TYPE_COUNT - 1)

#define MP_VALUE_GET_IMMEDIATE(value)	((uintptr_t)(value) >> 4)
#define MP_VALUE_GET_TYPE(value)	(((uintptr_t)(value) >> 2) & 0x3U)
#define MP_VALUE_GET_PTR(value)		((void *)((uintptr_t)(value) & ~0x3U))

#define MP_VALUE_SET_TYPE(value, type)	((mp_value_t)(((uintptr_t)(value) & ~0x0CU) | (type << 2)))

#define MP_VALUE_NEW_VALUE_PTR(uptr)	((mp_value_t)((uintptr_t)(uptr) | 0x0U))
#define MP_VALUE_NEW_OBJECT_PTR(uptr)	((mp_value_t)((uintptr_t)(uptr) | 0x1U))
#define MP_VALUE_NEW_OTHER_PTR(uptr)	((mp_value_t)((uintptr_t)(uptr) | 0x2U))
#define MP_VALUE_NEW_IMMEDIATE(type, uptr)                                                         \
	((mp_value_t)(((uintptr_t)(uptr) << 4) | ((type) << 2) | 0x3U))

#define MP_COMPARE(a, b)                                                                           \
	({                                                                                         \
		__typeof__(a) _a = (a);                                                            \
		__typeof__(b) _b = (b);                                                            \
		(_a < _b) ? MP_VALUE_LESS_THAN                                                     \
			  : ((_a > _b) ? MP_VALUE_GREATER_THAN : MP_VALUE_EQUAL);                  \
	})

#define MP_VALUE_RANGES_OVERLAP(ref_val, cmp_val)                                                  \
	!(MP_VALUE_RANGE(ref_val)->min > MP_VALUE_RANGE(cmp_val)->max ||                           \
	  MP_VALUE_RANGE(cmp_val)->min > MP_VALUE_RANGE(ref_val)->max)

#define MP_VALUE_NEW_INTERSECT_RANGE(ref_val, cmp_val)                                             \
	MP_VALUE_RANGES_OVERLAP(ref_val, cmp_val)                                                  \
	? mp_value_new(                                                                            \
		  MP_TYPE_RANGE,                                                                   \
		  MAX(MP_VALUE_RANGE(ref_val)->min, MP_VALUE_RANGE(cmp_val)->min),                 \
		  MIN(MP_VALUE_RANGE(ref_val)->max, MP_VALUE_RANGE(cmp_val)->max),                 \
		  sys_gcd(MP_VALUE_RANGE(ref_val)->step,                                           \
			  MP_VALUE_RANGE(compare_val)->step),                                      \
		  NULL)                                                                            \
	: NULL

struct mp_value_simple {
	struct mp_value base;
	union {
		int64_t v_int;
		const char *v_cstring;
	};
};

struct mp_value_list {
	struct mp_value base;
	sys_slist_t v_list;
};

struct mp_value_range {
	struct mp_value base;
	int64_t min;
	int64_t max;
	int64_t step;
};

struct mp_value_node {
	mp_value_t value;
	sys_snode_t node;
};

static const size_t mp_value_type_sizes[MP_TYPE_COUNT] = {
	[MP_TYPE_NONE] = sizeof(struct mp_value_simple),
	[MP_TYPE_BOOLEAN] = sizeof(struct mp_value_simple),
	[MP_TYPE_ENUM] = sizeof(struct mp_value_simple),
	[MP_TYPE_INT] = sizeof(struct mp_value_simple),
	[MP_TYPE_STRING] = sizeof(struct mp_value_simple),
	[MP_TYPE_RANGE] = sizeof(struct mp_value_range),
	[MP_TYPE_LIST] = sizeof(struct mp_value_list),
	[MP_TYPE_OBJECT] = 0,
	[MP_TYPE_PTR] = 0,
};

static const uint32_t mp_value_intersect_mask[MP_TYPE_COUNT] = {
	[MP_TYPE_NONE] = 0,
	[MP_TYPE_BOOLEAN] = BIT(MP_TYPE_BOOLEAN) | BIT(MP_TYPE_LIST),
	[MP_TYPE_ENUM] = BIT(MP_TYPE_ENUM) | BIT(MP_TYPE_LIST),
	[MP_TYPE_INT] = BIT(MP_TYPE_INT) | BIT(MP_TYPE_RANGE) | BIT(MP_TYPE_LIST),
	[MP_TYPE_STRING] = BIT(MP_TYPE_STRING) | BIT(MP_TYPE_LIST),
	[MP_TYPE_RANGE] = BIT(MP_TYPE_INT) | BIT(MP_TYPE_RANGE) | BIT(MP_TYPE_LIST),
	[MP_TYPE_LIST] = BIT(MP_TYPE_BOOLEAN) | BIT(MP_TYPE_ENUM) | BIT(MP_TYPE_INT) |
			 BIT(MP_TYPE_STRING) | BIT(MP_TYPE_RANGE) | BIT(MP_TYPE_LIST),
	[MP_TYPE_OBJECT] = 0,
	[MP_TYPE_PTR] = 0,
};

enum mp_value_type mp_value_get_type(const mp_value_t value)
{
	if (MP_VALUE_IS_IMMEDIATE(value)) {
		return MP_VALUE_GET_TYPE(value);
	} else if (MP_VALUE_IS_OBJECT_PTR(value)) {
		return MP_TYPE_OBJECT;
	} else if (MP_VALUE_IS_VALUE_PTR(value)) {
		return value->_type;
	} else {
		return MP_TYPE_NONE;
	}
}

void mp_value_set_type(mp_value_t *value, enum mp_value_type type)
{
	if (MP_VALUE_IS_IMMEDIATE(*value)) {
		*value = MP_VALUE_SET_TYPE(*value, type);
	} else if (MP_VALUE_IS_VALUE_PTR(*value)) {
		(*value)->_type = type;
	}
}

bool mp_value_is_primitive(const mp_value_t value)
{
	enum mp_value_type type = mp_value_get_type(value);

	if (MP_VALUE_IS_NULL(value) || !MP_VALUE_IS_VALID(value)) {
		return false;
	}

	return ((BIT(MP_TYPE_BOOLEAN) | BIT(MP_TYPE_ENUM) | BIT(MP_TYPE_INT) |
		 BIT(MP_TYPE_STRING)) &
		BIT(type)) != 0;
}

static int mp_value_set_range(mp_value_t value, int type, va_list *args)
{
	if (MP_VALUE_IS_NULL(value)) {
		return -EINVAL;
	}

	mp_value_set_type(&value, type);

	MP_VALUE_RANGE(value)->min = va_arg(*args, int64_t);
	MP_VALUE_RANGE(value)->max = va_arg(*args, int64_t);
	MP_VALUE_RANGE(value)->step = va_arg(*args, int64_t);

	return 0;
}

static int mp_value_set_list(mp_value_t value, va_list *args)
{
	mp_value_t list_item;
	int ret;

	mp_value_set_type(&value, MP_TYPE_LIST);

	while ((list_item = va_arg(*args, mp_value_t)) != NULL) {
		ret = mp_value_list_append(value, list_item);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int mp_value_set_immediate(mp_value_t *value, int64_t i)
{
	enum mp_value_type type = mp_value_get_type(*value);

	if (MP_VALUE_IS_FITTING(i)) {
		mp_value_destroy(*value);
		*value = MP_VALUE_NEW_IMMEDIATE(type, i);
		return 0;
	}

	if (MP_VALUE_IS_IMMEDIATE(*value)) {
		*value = k_calloc(1, mp_value_type_sizes[type]);
		if (*value == NULL) {
			LOG_ERR("Failed to allocate %u bytes to increase value storage size",
				mp_value_type_sizes[type]);
			return -ENOMEM;
		}
	}

	(*value)->_type = type;
	((struct mp_value_simple *)*value)->v_int = i;

	return 0;
}

static int mp_value_set_va_list(mp_value_t *value, int type, va_list *args)
{
	if (MP_VALUE_IS_NULL(value)) {
		return -EINVAL;
	}

	mp_value_set_type(value, type);

	switch (mp_value_get_type(*value)) {
	case MP_TYPE_BOOLEAN:
	case MP_TYPE_ENUM:
	case MP_TYPE_INT:
		return mp_value_set_immediate(value, va_arg(*args, int64_t));
	case MP_TYPE_STRING:
		MP_VALUE_SIMPLE(*value)->v_cstring = va_arg(*args, const char *);
		mp_value_set_type(value, MP_TYPE_STRING);
		return 0;;
	case MP_TYPE_PTR:
		*value = MP_VALUE_NEW_OTHER_PTR(va_arg(*args, void *));
		return 0;
	case MP_TYPE_OBJECT:
		struct mp_object * object = MP_VALUE_GET_PTR(value);
		mp_object_replace(&object, va_arg(*args, struct mp_object *));
		*value = MP_VALUE_NEW_OBJECT_PTR(object);
		return 0;
	case MP_TYPE_RANGE:
		return mp_value_set_range(*value, type, args);
	case MP_TYPE_LIST:
		return mp_value_set_list(*value, args);
	default:
		LOG_ERR("Unknown mp_value type: %d", mp_value_get_type(*value));
		return -EINVAL;
	}
}

int mp_value_set(mp_value_t *value, int type, ...)
{
	va_list args;
	int ret;

	va_start(args, type);

	ret = mp_value_set_va_list(value, type, &args);

	va_end(args);

	return ret;
}

int mp_value_set_value_ptr(mp_value_t value, int type, ...)
{
	va_list args;
	int ret;

	va_start(args, type);

	ret = mp_value_set_va_list(&value, type, &args);

	va_end(args);

	return ret;
}

const char *mp_value_get_string(const mp_value_t value)
{
	return MP_VALUE_SIMPLE_CONST(value)->v_cstring;
}

int64_t mp_value_get_int(const mp_value_t value)
{
	if (MP_VALUE_IS_IMMEDIATE(value)) {
		return MP_VALUE_GET_IMMEDIATE(value);
	}
	return MP_VALUE_SIMPLE_CONST(value)->v_int;
}

void *mp_value_get_ptr(const mp_value_t value)
{
	return MP_VALUE_GET_PTR(value);
}

bool mp_value_get_boolean(const mp_value_t value)
{
	return MP_VALUE_GET_IMMEDIATE(value);
}

mp_value_t mp_value_new_empty(enum mp_value_type type)
{
	mp_value_t value;

	if (!IN_RANGE(type, MP_TYPE_NONE + 1, MP_TYPE_COUNT - 1)) {
		LOG_ERR("Invalid value type: %d", type);
		return NULL;
	}

	switch (type) {
	case MP_TYPE_BOOLEAN:
	case MP_TYPE_ENUM:
	case MP_TYPE_INT:
		return (mp_value_t)MP_VALUE_NEW_IMMEDIATE(type, 0);
	case MP_TYPE_OBJECT:
		return (mp_value_t)MP_VALUE_NEW_OBJECT_PTR(NULL);
	case MP_TYPE_PTR:
		return (mp_value_t)MP_VALUE_NEW_OTHER_PTR(NULL);
	default:
		break;
	}

	value = k_calloc(1, mp_value_type_sizes[type]);
	if (value == NULL) {
		LOG_ERR("Failed to mp_value_t of type %d", type);
		return NULL;
	}

	mp_value_set_type(&value, type);

	if (type == MP_TYPE_LIST) {
		sys_slist_init(&MP_VALUE_LIST(value)->v_list);
	}

	return value;
}

int mp_value_destroy(mp_value_t value)
{
	struct mp_value_node *value_node;
	sys_snode_t *node;

	if (MP_VALUE_IS_NULL(value) || MP_VALUE_IS_IMMEDIATE(value)) {
		return -EINVAL;
	}

	if (mp_value_get_type(value) == MP_TYPE_LIST) {
		while (!sys_slist_is_empty(&MP_VALUE_LIST(value)->v_list)) {
			node = sys_slist_get(&MP_VALUE_LIST(value)->v_list);
			if (MP_VALUE_IS_NULL(node)) {
				k_free(value);
				return -EIO;
			}

			value_node = CONTAINER_OF(node, struct mp_value_node, node);
			mp_value_destroy(value_node->value);
			k_free(value_node);
		}
	}

	if (MP_VALUE_IS_OBJECT_PTR(value)) {
		mp_object_unref(MP_VALUE_GET_PTR(value));
	}

	if (!MP_VALUE_IS_IMMEDIATE(value)) {
		k_free(MP_VALUE_GET_PTR(value));
	}

	return 0;
}

mp_value_t mp_value_new(enum mp_value_type type, ...)
{

	mp_value_t value;
	va_list args;

	va_start(args, type);

	value = mp_value_new_va_list(type, &args);

	va_end(args);

	return value;
}

mp_value_t mp_value_new_va_list(enum mp_value_type type, va_list *args)
{
	int ret;
	mp_value_t value = mp_value_new_empty(type);

	if (MP_VALUE_IS_NULL(value)) {
		return NULL;
	}

	ret = mp_value_set_va_list(&value, type, args);
	if (ret < 0) {
		LOG_ERR("Failed to set mp_value_t of type %d: error %d", type, ret);
		mp_value_destroy(value);
		return NULL;
	}

	return value;
}

static int mp_value_copy(mp_value_t *dst, const mp_value_t src)
{
	int ret;

	if (MP_VALUE_IS_NULL(dst) || MP_VALUE_IS_NULL(src)) {
		return -EINVAL;
	}

	if (mp_value_get_type(src) == MP_TYPE_LIST) {
		struct mp_value_node *v_node;

		SYS_SLIST_FOR_EACH_CONTAINER(&MP_VALUE_LIST(src)->v_list, v_node, node) {
			mp_value_t dup = mp_value_duplicate(v_node->value);

			if (MP_VALUE_IS_NULL(dup)) {
				return -ENOMEM;
			}

			ret = mp_value_list_append(*dst, dup);
			if (ret < 0) {
				LOG_ERR("Failed to append value to the list");
				mp_value_destroy(dup);
				return ret;
			}
		}
	} else if (MP_VALUE_IS_OBJECT_PTR(src)) {
		*dst = MP_VALUE_NEW_OBJECT_PTR(MP_VALUE_GET_PTR(src));
		mp_object_ref(MP_VALUE_GET_PTR(dst));
	} else if (MP_VALUE_IS_VALUE_PTR(src)) {
		if (MP_VALUE_IS_IMMEDIATE(*dst)) {
			*dst = k_calloc(1, mp_value_type_sizes[mp_value_get_type(src)]);
			if (*dst == NULL) {
				return -ENOMEM;
			}
		}
		memcpy(*dst, src, mp_value_type_sizes[mp_value_get_type(src)]);
	} else if (MP_VALUE_IS_OTHER_PTR(src)) {
		*dst = MP_VALUE_NEW_OTHER_PTR(MP_VALUE_GET_PTR(src));
	} else if (MP_VALUE_IS_IMMEDIATE(src)) {
		*dst = MP_VALUE_NEW_IMMEDIATE(MP_VALUE_GET_TYPE(src), MP_VALUE_GET_IMMEDIATE(src));
	} else {
		return -EINVAL;
	}

	return 0;
}

mp_value_t mp_value_duplicate(const mp_value_t value)
{
	mp_value_t dup_value;
	int ret;

	if (MP_VALUE_IS_NULL(value)) {
		return NULL;
	}

	dup_value = mp_value_new_empty(mp_value_get_type(value));
	if (MP_VALUE_IS_NULL(dup_value)) {
		return NULL;
	}

	ret = mp_value_copy(&dup_value, value);
	if (ret < 0) {
		LOG_ERR("Failed to copy mp_value_t: %d", ret);
		mp_value_destroy(dup_value);
		return NULL;
	}

	return dup_value;
}

int mp_value_list_append(mp_value_t list, mp_value_t append_value)
{
	struct mp_value_node *node;

	if (MP_VALUE_IS_NULL(list) || MP_VALUE_IS_NULL(append_value)) {
		return -EINVAL;
	}

	node = k_calloc(1, sizeof(struct mp_value_node));
	if (node == NULL) {
		return -ENOMEM;
	}

	node->value = append_value;
	sys_slist_append(&MP_VALUE_LIST(list)->v_list, &node->node);

	return 0;
}

mp_value_t mp_value_list_get(const mp_value_t list, int index)
{
	sys_snode_t *node;
	struct mp_value_node *value_node = NULL;
	int count = 0;

	SYS_SLIST_FOR_EACH_NODE((sys_slist_t *)&MP_VALUE_LIST(list)->v_list, node) {
		if (count++ == index) {
			value_node = CONTAINER_OF(node, struct mp_value_node, node);
			break;
		}
	}

	return (value_node != NULL) ? value_node->value : NULL;
}

bool mp_value_list_is_empty(const mp_value_t list)
{
	return sys_slist_is_empty(&MP_VALUE_LIST_CONST(list)->v_list);
}

size_t mp_value_list_get_size(const mp_value_t list)
{
	return sys_slist_len(&MP_VALUE_LIST_CONST(list)->v_list);
}

int mp_value_get_range_min(const mp_value_t range)
{
	return MP_VALUE_RANGE_CONST(range)->min;
}

int mp_value_get_range_max(const mp_value_t range)
{
	return MP_VALUE_RANGE_CONST(range)->max;
}

int mp_value_get_range_step(const mp_value_t range)
{
	return MP_VALUE_RANGE_CONST(range)->step;
}

struct mp_object *mp_value_get_object(mp_value_t value)
{
	return MP_VALUE_GET_PTR(value);
}

static int mp_value_list_compare(const mp_value_t list1, const mp_value_t list2);

int mp_value_compare(const mp_value_t val1, const mp_value_t val2)
{
	if (MP_VALUE_IS_NULL(val1) || MP_VALUE_IS_NULL(val2)) {
		return MP_VALUE_COMPARE_FAILED;
	}

	if (mp_value_get_type(val1) != mp_value_get_type(val2)) {
		return MP_VALUE_COMPARE_FAILED;
	}

	switch (mp_value_get_type(val1)) {
	case MP_TYPE_BOOLEAN:
	case MP_TYPE_ENUM:
		return mp_value_get_int(val1) == mp_value_get_int(val2)
			       ? MP_VALUE_EQUAL
			       : MP_VALUE_UNORDERED;
	case MP_TYPE_INT:
		return MP_COMPARE(mp_value_get_int(val1), mp_value_get_int(val2));
	case MP_TYPE_STRING:
		return strcmp(MP_VALUE_SIMPLE_CONST(val1)->v_cstring,
			      MP_VALUE_SIMPLE_CONST(val2)->v_cstring) == 0
			       ? MP_VALUE_EQUAL
			       : MP_VALUE_UNORDERED;
	case MP_TYPE_RANGE:
		return (MP_VALUE_RANGE_CONST(val1)->min == MP_VALUE_RANGE_CONST(val2)->min &&
			MP_VALUE_RANGE_CONST(val1)->max == MP_VALUE_RANGE_CONST(val2)->max &&
			MP_VALUE_RANGE_CONST(val1)->step == MP_VALUE_RANGE_CONST(val2)->step)
			       ? MP_VALUE_EQUAL
			       : MP_VALUE_UNORDERED;
	case MP_TYPE_LIST:
		return mp_value_list_compare(val1, val2);
	default:
		return MP_VALUE_COMPARE_FAILED;
	}
}

static int mp_value_list_compare(const mp_value_t list1, const mp_value_t list2)
{
	int size1 = mp_value_list_get_size(list1);
	int size2 = mp_value_list_get_size(list2);
	int count_matched = 0;
	struct mp_value_node *v_node1, *v_node2;

	if (mp_value_get_type(list1) != MP_TYPE_LIST || mp_value_get_type(list2) != MP_TYPE_LIST) {
		return MP_VALUE_COMPARE_FAILED;
	}

	if (size1 != size2) {
		return MP_VALUE_UNORDERED;
	}

	SYS_SLIST_FOR_EACH_CONTAINER((sys_slist_t *)&MP_VALUE_LIST(list1)->v_list, v_node1, node) {
		SYS_SLIST_FOR_EACH_CONTAINER((sys_slist_t *)&MP_VALUE_LIST(list2)->v_list, v_node2,
					     node) {
			if (mp_value_compare(v_node1->value, v_node2->value) == MP_VALUE_EQUAL) {
				count_matched++;
			}
		}
	}

	return count_matched == size1 ? MP_VALUE_EQUAL : MP_VALUE_UNORDERED;
}

bool mp_value_can_intersect(const mp_value_t val1, const mp_value_t val2)
{
	if (MP_VALUE_IS_NULL(val1) || MP_VALUE_IS_NULL(val2) ||
	    !MP_VALUE_IS_VALID(val1) || !MP_VALUE_IS_VALID(val2)) {
		return false;
	}
	return (mp_value_intersect_mask[mp_value_get_type(val1)] &
		BIT(mp_value_get_type(val2))) != 0;
}

mp_value_t mp_value_intersect_range(const mp_value_t ref_val,
				    const mp_value_t compare_val)
{
	mp_value_t intersect_value;

	if (mp_value_get_type(compare_val) == MP_TYPE_RANGE &&
	    mp_value_get_type(ref_val) == MP_TYPE_RANGE) {
		intersect_value = MP_VALUE_NEW_INTERSECT_RANGE(ref_val, compare_val);
	} else if (mp_value_get_type(ref_val) == MP_TYPE_RANGE &&
		   mp_value_get_type(compare_val) == MP_TYPE_INT &&
		   IN_RANGE(mp_value_get_int(compare_val),                                                                   \
			    MP_VALUE_RANGE(ref_val)->min, MP_VALUE_RANGE(ref_val)->max)) {
		intersect_value = mp_value_new(mp_value_get_type(compare_val),
					       mp_value_get_int(compare_val), NULL);
	} else {
		intersect_value = NULL;
	}

	return intersect_value;
}

mp_value_t mp_value_intersect_list(const mp_value_t list,
					 const mp_value_t compare_val)
{
	mp_value_t intersect_value = NULL;
	mp_value_t intersect_list = NULL;
	struct mp_value_node *v_node1, *v_node2;

	if (MP_VALUE_IS_NULL(list) || MP_VALUE_IS_NULL(compare_val) ||
	    !MP_VALUE_IS_VALID(list) || !MP_VALUE_IS_VALID(compare_val)) {
		return NULL;
	}

	intersect_list = mp_value_new_empty(MP_TYPE_LIST);
	if (intersect_list == NULL) {
		LOG_ERR("Failed to allocate result list");
		return NULL;
	}

	SYS_SLIST_FOR_EACH_CONTAINER((sys_slist_t *)&MP_VALUE_LIST(list)->v_list, v_node1, node) {
		intersect_value = NULL;

		switch (mp_value_get_type(compare_val)) {
		case MP_TYPE_BOOLEAN:
		case MP_TYPE_ENUM:
		case MP_TYPE_INT:
		case MP_TYPE_STRING:
			if (mp_value_compare(compare_val, v_node1->value) == MP_VALUE_EQUAL) {
				intersect_value = mp_value_duplicate(compare_val);
			}
			break;
		case MP_TYPE_RANGE:
			intersect_value = mp_value_intersect_range(compare_val, v_node1->value);
			break;
		case MP_TYPE_LIST:
			SYS_SLIST_FOR_EACH_CONTAINER(
				(sys_slist_t *)&MP_VALUE_LIST(compare_val)->v_list, v_node2, node) {
				if (mp_value_compare(v_node1->value, v_node2->value) ==
				    MP_VALUE_EQUAL) {
					intersect_value = mp_value_duplicate(v_node2->value);
					if (intersect_value == NULL) {
						LOG_ERR("Failed to allocate intersection value");
						goto error;
					}

					mp_value_list_append(intersect_list, intersect_value);
					intersect_value = NULL;
					break;
				}
			}
			break;
		default:
			break;
		}

		if (intersect_value != NULL) {
			mp_value_list_append(intersect_list, intersect_value);
		}
	}

	if (mp_value_list_get_size(intersect_list) == 0) {
		LOG_WRN("No intersection between %p and %p", list, compare_val);
		goto error;
	}

	return intersect_list;

error:
	mp_value_destroy(intersect_list);
	return NULL;
}

mp_value_t mp_value_intersect(const mp_value_t val1, const mp_value_t val2)
{
	mp_value_t ref_val, compare_val;
	mp_value_t intersect_val = NULL;

	/* Check if intersect */
	if (!mp_value_can_intersect(val1, val2)) {
		return NULL;
	}

	/* When two values don't have the same type */
	if (mp_value_get_type(val1) >= mp_value_get_type(val2)) {
		ref_val = val1;
		compare_val = val2;
	} else {
		ref_val = val2;
		compare_val = val1;
	}

	if (mp_value_is_primitive(ref_val)) {
		if (mp_value_compare(val1, val2) == MP_VALUE_EQUAL) {
			intersect_val = mp_value_duplicate(val1);
		}
	} else {
		switch (mp_value_get_type(ref_val)) {
		case MP_TYPE_RANGE:
			intersect_val = mp_value_intersect_range(ref_val, compare_val);
			break;
		case MP_TYPE_LIST:
			intersect_val = mp_value_intersect_list(ref_val, compare_val);
			break;
		default:
			break;
		}
	}

	return intersect_val;
}

static inline void mp_value_print_int(const mp_value_t value)
{
	printk("%d", mp_value_get_int(value));
}

static inline void mp_value_print_string(const mp_value_t value)
{
	printk("%s", mp_value_get_string(value));
}

static inline void mp_value_print_int_range(const mp_value_t value)
{
	printk("[%d, %d, %d]", MP_VALUE_RANGE_CONST(value)->min,
	       MP_VALUE_RANGE_CONST(value)->max, MP_VALUE_RANGE_CONST(value)->step);
}

static inline void mp_value_print_list(const mp_value_t value)
{
	struct mp_value_node *value_node;

	printk("{");
	SYS_SLIST_FOR_EACH_CONTAINER((sys_slist_t *)&MP_VALUE_LIST(value)->v_list, value_node,
				     node) {
		mp_value_print(value_node->value, false);
		if (sys_slist_peek_next(&value_node->node) != NULL) {

			printk(", ");
		}
	}
	printk("}");
}

void mp_value_print(const mp_value_t value, bool new_line)
{
	typedef void (*mp_value_print_fn)(const mp_value_t);
	static const mp_value_print_fn mp_value_print_table[MP_TYPE_COUNT] = {
		[MP_TYPE_NONE] = NULL,
		[MP_TYPE_BOOLEAN] = mp_value_print_int,
		[MP_TYPE_ENUM] = mp_value_print_int,
		[MP_TYPE_INT] = mp_value_print_int,
		[MP_TYPE_RANGE] = mp_value_print_int_range,
		[MP_TYPE_STRING] = mp_value_print_string,
		[MP_TYPE_LIST] = mp_value_print_list,
		[MP_TYPE_OBJECT] = NULL,
		[MP_TYPE_PTR] = NULL,
	};

	if (MP_VALUE_IS_NULL(value) || !MP_VALUE_IS_VALID(value) ||
	    mp_value_print_table[mp_value_get_type(value)] == NULL) {
		LOG_ERR("Invalid mp_value %p to print, type %u", value, mp_value_get_type(value));
		return;
	}

	mp_value_print_fn print_fn = mp_value_print_table[mp_value_get_type(value)];

	if (print_fn != NULL) {
		print_fn(value);
	}

	if (new_line) {
		printk("\n");
	}
}
