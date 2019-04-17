/** @file list.c - Implementation
 ** @brief Railway monitor list operation.
 ** @author Zhiwei Zeng
 ** @date 2018.08.14
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "list.h"

/** @brief Initialize list.
 ** @param list list.
 **/
void ListInit(List *list)
{
	list->head = NULL;
	list->tail = NULL;
	list->size = 0;
}

/** @brief Allocate memory for value
 ** @param len length of the buffer.
 ** @return allocated buffer.
 **/
void *ListAlloc(size_t len)
{
	return malloc(len);
}

/** @brief Add node to the list tail.
 ** @param list list.
 ** @paramm val node value.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int ListAddTail(List *list, void *val)
{
	struct Node *node = (struct Node *)malloc(sizeof(struct Node));
	if (!node) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	node->val = val;
	node->next = NULL;
	
	if (!list->head) {
		list->head = node;
		list->tail = node;
		list->size++;
		return 0;
	}
	
	list->tail->next = node;
	list->tail = node;
	list->size++;
	
	return 0;
}

/** @brief Concatenate the second list to the first list.
 ** @param first first list.
 ** @param second second list.
 **/
void ListConcatenate(List *first, List *second)
{
	if (second->head) {
		if (first->head) {
			first->tail->next = second->head;
			first->tail = second->tail;
			first->size += second->size;
		} else {
			first->head = second->head;
			first->tail = second->tail;
			first->size = second->size;
		}
	}
}

/** @brief Copy a list to another list.
 ** @param src source list.
 ** @param dst destination list.
 ** @param bpnv bytes per node value.
 **/
void ListCopy(List *src, List *dst, size_t bpnv)
{
	if (!src || !dst) {
		return;
	}
	
	struct Node *node = src->head;	
	while (node) {
		void *val = ListAlloc(bpnv);
		if (!val) exit(-1);
		memcpy(val, node->val, bpnv);
		ListAddTail(dst, val);
		node = node->next;
	}
}

/** @brief Delete a node from list.
 ** @param list list.
 ** @param node a node.
 ** @return the next node of deleted node.
 **/
struct Node *ListDelNode(List *list, struct Node *node)
{
	if (!list) {
		return NULL;
	}
	
	struct Node *head = list->head;
	if (!head) {
		return NULL;
	}
	
	if (node == head) {
		list->head = head->next;
		free(node->val);
		free(node);
		node = NULL;
		list->size -= 1;
		return list->head;
	}
	
	struct Node *prev = head;
	while (head && head != node) {
		prev = head;
		head = head->next;
	}
	
	if (prev->next != node) {
		fprintf(stderr, "not find the node[%s:%d].\n", __FILE__, __LINE__);
		return NULL;
	}
	
	prev->next = node->next;
	free(node->val);
	free(node);
	node = NULL;
	list->size -= 1;
	return prev->next;
}

/** @brief Delete all nodes of list.
 ** @param list list.
 **/
void ListDelAll(List *list)
{
	if (!list) {
		return;
	}
	
	struct Node *head = list->head;
	if (!head) {
		return;
	}
	
	while (head) {
		struct Node *node = head;
		head = head->next;
		free(node->val);
		free(node);
		node = NULL;
	}
}